//! Aerial survey → draft map → mode-typed A* → ground execution with local map correction.
//!
//! A 2D grid prototype of the hybrid robot's planning pipeline:
//!
//! 1. **Aerial survey**: a downward elevation sensor sees large geometry
//!    (walls, water/ditches) but cannot see under tree canopy and misses
//!    sub-resolution obstacles (rocks) and non-geometric hazards (mud).
//!    Result: a *draft* belief map of Free / Blocked / Unknown cells.
//! 2. **Global plan**: A* over `(cell, mode)` states. Every step is typed
//!    `Drive` or `Fly`; switching modes costs a takeoff/landing penalty and is
//!    only allowed on a cell believed Free (verified takeoff/landing zone).
//! 3. **Execution**: the ground robot senses a short radius around itself.
//!    When ground truth contradicts the draft, the map is corrected and the
//!    path is repaired *locally* (detour that rejoins the global plan). A full
//!    global re-plan is only the fallback.
//! 4. Every takeoff and landing goes through the safety [`Supervisor`].

use crate::supervisor::{Event, Mode, State, Supervisor};
use crate::watchdog::Source;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

// ------------------------------------------------------------------ world

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Terrain {
    Free,
    Wall,
    Water,
    Rock,
    Mud,
}

impl Terrain {
    pub fn drivable(self) -> bool {
        matches!(self, Terrain::Free)
    }
}

/// Ground truth. Unknown to the robot except through its sensors.
pub struct World {
    pub w: usize,
    pub h: usize,
    terrain: Vec<Terrain>,
    canopy: Vec<bool>,
    pub start: (usize, usize),
    pub goal: (usize, usize),
}

impl World {
    pub fn terrain(&self, x: usize, y: usize) -> Terrain {
        self.terrain[y * self.w + x]
    }
    pub fn canopy(&self, x: usize, y: usize) -> bool {
        self.canopy[y * self.w + x]
    }
    fn paint(&mut self, x0: usize, y0: usize, x1: usize, y1: usize, t: Terrain) {
        for y in y0..=y1 {
            for x in x0..=x1 {
                self.terrain[y * self.w + x] = t;
            }
        }
    }

    /// Hand-built test site: a river that can't be driven across, a building,
    /// a mud patch and a rock the drone can't see, and a tree grove hiding
    /// a rock and a fallen trunk under its canopy.
    pub fn demo() -> Self {
        let (w, h) = (48, 16);
        let mut wd = World {
            w,
            h,
            terrain: vec![Terrain::Free; w * h],
            canopy: vec![false; w * h],
            start: (2, 8),
            goal: (45, 8),
        };
        for y in 0..h {
            let x0 = 24 + y / 4;
            wd.paint(x0, y, x0 + 1, y, Terrain::Water);
        }
        wd.paint(10, 1, 13, 4, Terrain::Wall); // building
        wd.paint(6, 7, 8, 9, Terrain::Mud); // mud patch
        wd.paint(16, 8, 16, 8, Terrain::Rock); // rock in open field
        for y in 5..=11 {
            for x in 33..=37 {
                wd.canopy[y * w + x] = true; // tree grove
            }
        }
        wd.paint(35, 8, 35, 8, Terrain::Rock); // rock under canopy
        wd.paint(34, 6, 36, 6, Terrain::Wall); // fallen trunk under canopy
        wd
    }
}

// ------------------------------------------------------------------ sensing and belief

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Belief {
    Free,
    Blocked,
    Unknown,
}

/// Downward elevation sensor at survey altitude.
pub fn aerial_observe(world: &World, x: usize, y: usize) -> Belief {
    if world.canopy(x, y) {
        return Belief::Unknown; // can't see the ground under trees
    }
    match world.terrain(x, y) {
        Terrain::Wall | Terrain::Water => Belief::Blocked, // big geometry: visible
        Terrain::Rock | Terrain::Mud | Terrain::Free => Belief::Free, // too small / not geometric
    }
}

/// Short-range ground LiDAR + camera: sees true traversability.
pub fn ground_observe(world: &World, x: usize, y: usize) -> Belief {
    if world.terrain(x, y).drivable() {
        Belief::Free
    } else {
        Belief::Blocked
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Correction {
    pub x: usize,
    pub y: usize,
    pub was: Belief,
    pub now: Belief,
}

#[derive(Clone)]
pub struct Map {
    pub w: usize,
    pub h: usize,
    cells: Vec<Belief>,
    corrected: Vec<bool>,
}

impl Map {
    /// Lawnmower survey over the whole site: every cell gets one aerial reading.
    pub fn from_survey(world: &World) -> Self {
        let mut cells = vec![Belief::Unknown; world.w * world.h];
        for y in 0..world.h {
            for x in 0..world.w {
                cells[y * world.w + x] = aerial_observe(world, x, y);
            }
        }
        Map { w: world.w, h: world.h, cells, corrected: vec![false; world.w * world.h] }
    }

    pub fn get(&self, x: usize, y: usize) -> Belief {
        self.cells[y * self.w + x]
    }
    pub fn was_corrected(&self, x: usize, y: usize) -> bool {
        self.corrected[y * self.w + x]
    }

    /// Fuse ground observations within `radius`; returns every cell that changed.
    pub fn ground_update(&mut self, world: &World, cx: usize, cy: usize, radius: usize) -> Vec<Correction> {
        let mut out = Vec::new();
        let (x0, x1) = (cx.saturating_sub(radius), (cx + radius).min(self.w - 1));
        let (y0, y1) = (cy.saturating_sub(radius), (cy + radius).min(self.h - 1));
        for y in y0..=y1 {
            for x in x0..=x1 {
                let now = ground_observe(world, x, y);
                let i = y * self.w + x;
                if self.cells[i] != now {
                    out.push(Correction { x, y, was: self.cells[i], now });
                    self.cells[i] = now;
                    self.corrected[i] = true;
                }
            }
        }
        out
    }
}

// ------------------------------------------------------------------ mode-typed A*

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Step {
    pub x: usize,
    pub y: usize,
    pub mode: Mode,
}

/// Costs in tenths (straight move = 10 units, diagonal = 14).
#[derive(Debug, Clone, Copy)]
pub struct Costs {
    pub drive_free: u32,
    pub drive_unknown: u32,
    pub fly: u32,
    /// One takeoff or one landing.
    pub mode_switch: u32,
}

impl Default for Costs {
    fn default() -> Self {
        Costs { drive_free: 10, drive_unknown: 12, fly: 40, mode_switch: 150 }
    }
}

fn mode_idx(m: Mode) -> usize {
    match m {
        Mode::Drive => 0,
        Mode::Fly => 1,
    }
}

fn octile(ax: usize, ay: usize, bx: usize, by: usize) -> u32 {
    let dx = ax.abs_diff(bx) as u32;
    let dy = ay.abs_diff(by) as u32;
    10 * dx.max(dy) + 4 * dx.min(dy)
}

pub struct PlanRequest {
    pub start: Step,
    pub goal: (usize, usize),
    pub drive_only: bool,
    /// Restrict the search to a window (x0, y0, x1, y1) for local repair.
    pub window: Option<(usize, usize, usize, usize)>,
}

/// A* over (x, y, mode). Ends at the goal in Drive mode.
pub fn plan(map: &Map, c: &Costs, req: &PlanRequest) -> Option<(Vec<Step>, u32)> {
    let (w, h) = (map.w, map.h);
    let idx = |x: usize, y: usize, m: Mode| (y * w + x) * 2 + mode_idx(m);
    let inside = |x: usize, y: usize| match req.window {
        Some((x0, y0, x1, y1)) => x >= x0 && x <= x1 && y >= y0 && y <= y1,
        None => true,
    };
    let drive_cost = |x: usize, y: usize| match map.get(x, y) {
        Belief::Free => Some(c.drive_free),
        Belief::Unknown => Some(c.drive_unknown),
        Belief::Blocked => None,
    };

    let n = w * h * 2;
    let mut g = vec![u32::MAX; n];
    let mut parent = vec![usize::MAX; n];
    let mut heap = BinaryHeap::new();
    let s = req.start;
    let si = idx(s.x, s.y, s.mode);
    g[si] = 0;
    heap.push(Reverse((octile(s.x, s.y, req.goal.0, req.goal.1), si)));

    while let Some(Reverse((_, cur))) = heap.pop() {
        let cell = cur / 2;
        let (x, y) = (cell % w, cell / w);
        let mode = if cur % 2 == 0 { Mode::Drive } else { Mode::Fly };
        if (x, y) == req.goal && mode == Mode::Drive {
            let mut path = Vec::new();
            let mut k = cur;
            while k != usize::MAX {
                let cl = k / 2;
                path.push(Step {
                    x: cl % w,
                    y: cl / w,
                    mode: if k % 2 == 0 { Mode::Drive } else { Mode::Fly },
                });
                k = parent[k];
            }
            path.reverse();
            return Some((path, g[cur]));
        }
        let gc = g[cur];
        let mut relax = |nx: usize, ny: usize, nm: Mode, cost: u32, heap: &mut BinaryHeap<_>| {
            let ni = idx(nx, ny, nm);
            let ng = gc + cost;
            if ng < g[ni] {
                g[ni] = ng;
                parent[ni] = cur;
                heap.push(Reverse((ng + octile(nx, ny, req.goal.0, req.goal.1), ni)));
            }
        };

        // Mode switch in place: only on a cell believed Free (verified zone).
        if !req.drive_only && map.get(x, y) == Belief::Free {
            let other = if mode == Mode::Drive { Mode::Fly } else { Mode::Drive };
            relax(x, y, other, c.mode_switch, &mut heap);
        }

        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                if nx < 0 || ny < 0 || nx >= w as i32 || ny >= h as i32 {
                    continue;
                }
                let (nx, ny) = (nx as usize, ny as usize);
                if !inside(nx, ny) {
                    continue;
                }
                let diag = dx != 0 && dy != 0;
                let scale = if diag { 14 } else { 10 };
                match mode {
                    Mode::Fly => relax(nx, ny, Mode::Fly, c.fly * scale / 10, &mut heap),
                    Mode::Drive => {
                        let Some(base) = drive_cost(nx, ny) else { continue };
                        // no cutting corners past blocked cells
                        if diag
                            && (map.get(nx, y) == Belief::Blocked || map.get(x, ny) == Belief::Blocked)
                        {
                            continue;
                        }
                        relax(nx, ny, Mode::Drive, base * scale / 10, &mut heap);
                    }
                }
            }
        }
    }
    None
}

/// Contiguous (mode, first index, last index) runs of a path.
pub fn segments(path: &[Step]) -> Vec<(Mode, usize, usize)> {
    let mut out: Vec<(Mode, usize, usize)> = Vec::new();
    for (i, s) in path.iter().enumerate() {
        match out.last_mut() {
            Some((m, _, end)) if *m == s.mode => *end = i,
            _ => out.push((s.mode, i, i)),
        }
    }
    out
}

// ------------------------------------------------------------------ execution

#[derive(Debug, Clone, PartialEq)]
pub enum MissionEvent {
    Survey { blocked: usize, unknown: usize },
    GlobalPlan { cost: u32, drive_steps: usize, fly_steps: usize, switches: usize },
    Hazard { step: usize, corrections: Vec<Correction> },
    UnknownsResolved { step: usize, count: usize },
    LocalRepair { step: usize, detour_len: usize, rejoined_at: (usize, usize) },
    GlobalReplan { step: usize, cost: u32 },
    Supervisor(String),
    Arrived { step: usize },
    Aborted { step: usize, why: String },
}

pub struct MissionReport {
    pub draft: Map,
    pub final_map: Map,
    pub global_plan: Vec<Step>,
    pub trail: Vec<Step>,
    pub events: Vec<MissionEvent>,
    pub planned_cost: u32,
    pub executed_cost: u32,
    pub local_repairs: usize,
    pub global_replans: usize,
    pub arrived: bool,
}

pub struct MissionConfig {
    pub sense_radius: usize,
    pub lookahead: usize,
    pub repair_pad: usize,
    pub costs: Costs,
}

impl Default for MissionConfig {
    fn default() -> Self {
        MissionConfig { sense_radius: 2, lookahead: 6, repair_pad: 4, costs: Costs::default() }
    }
}

fn step_cost(map: &Map, c: &Costs, a: Step, b: Step) -> u32 {
    if a.mode != b.mode {
        return c.mode_switch;
    }
    let scale = if a.x != b.x && a.y != b.y { 14 } else { 10 };
    let base = match b.mode {
        Mode::Fly => c.fly,
        Mode::Drive => match map.get(b.x, b.y) {
            Belief::Unknown => c.drive_unknown,
            _ => c.drive_free,
        },
    };
    base * scale / 10
}

const SOURCES: [Source; 5] = [
    Source::Planner,
    Source::Localization,
    Source::Lidar,
    Source::DriveController,
    Source::FlightController,
];

pub fn run_mission(world: &World, sup: &mut Supervisor, cfg: &MissionConfig) -> MissionReport {
    let mut events = Vec::new();
    let mut t: u64 = 0;
    let sup_log = |sup: &mut Supervisor, ev: &mut Vec<MissionEvent>| {
        for l in sup.drain_log() {
            ev.push(MissionEvent::Supervisor(l));
        }
    };

    // 1. survey
    let draft = Map::from_survey(world);
    let mut map = draft.clone();
    let (mut blocked, mut unknown) = (0, 0);
    for y in 0..map.h {
        for x in 0..map.w {
            match map.get(x, y) {
                Belief::Blocked => blocked += 1,
                Belief::Unknown => unknown += 1,
                Belief::Free => {}
            }
        }
    }
    events.push(MissionEvent::Survey { blocked, unknown });

    // 2. global plan
    let start = Step { x: world.start.0, y: world.start.1, mode: Mode::Drive };
    let req = PlanRequest { start, goal: world.goal, drive_only: false, window: None };
    let Some((global, planned_cost)) = plan(&map, &cfg.costs, &req) else {
        events.push(MissionEvent::Aborted { step: 0, why: "no global plan".into() });
        return MissionReport {
            draft, final_map: map, global_plan: vec![], trail: vec![], events,
            planned_cost: 0, executed_cost: 0, local_repairs: 0, global_replans: 0, arrived: false,
        };
    };
    let segs = segments(&global);
    events.push(MissionEvent::GlobalPlan {
        cost: planned_cost,
        drive_steps: global.iter().filter(|s| s.mode == Mode::Drive).count(),
        fly_steps: global.iter().filter(|s| s.mode == Mode::Fly).count(),
        switches: segs.len().saturating_sub(1),
    });

    // 3. arm + engage through the supervisor
    for s in SOURCES {
        sup.watchdog.register(s, 200, t);
    }
    sup.interlocks.at_rest = true;
    let _ = sup.handle(Event::Arm, t);
    let _ = sup.handle(Event::EngageAuto, t);
    sup_log(sup, &mut events);

    let mut path = global.clone();
    let mut i = 0usize;
    let mut trail = vec![start];
    let mut executed_cost = 0u32;
    let (mut local_repairs, mut global_replans) = (0usize, 0usize);
    let mut guard = 0;

    loop {
        guard += 1;
        if guard > 5_000 {
            events.push(MissionEvent::Aborted { step: i, why: "step limit".into() });
            break;
        }
        let cur = path[i];

        // sense (only on the ground)
        if cur.mode == Mode::Drive {
            let corr = map.ground_update(world, cur.x, cur.y, cfg.sense_radius);
            let hazards: Vec<Correction> = corr.iter().copied().filter(|c| c.now == Belief::Blocked).collect();
            let resolved = corr.iter().filter(|c| c.was == Belief::Unknown && c.now == Belief::Free).count();
            if !hazards.is_empty() {
                events.push(MissionEvent::Hazard { step: trail.len() - 1, corrections: hazards });
            }
            if resolved > 0 {
                events.push(MissionEvent::UnknownsResolved { step: trail.len() - 1, count: resolved });
            }
        }

        // validate the next few steps against the corrected map
        let end = (i + 1 + cfg.lookahead).min(path.len());
        let bad = (i + 1..end).find(|&k| {
            let s = path[k];
            let switching = path[k - 1].mode != s.mode;
            (s.mode == Mode::Drive && map.get(s.x, s.y) == Belief::Blocked)
                || (switching && map.get(s.x, s.y) != Belief::Free)
        });

        if let Some(k) = bad {
            // local repair: detour (drive only, inside a window) that rejoins the global plan
            // rejoin a few cells past the hazard, not right at its edge
            let rejoin = ((k + 3).min(path.len() - 1)..path.len()).find(|&j| {
                path[j].mode == Mode::Drive
                    && path[j - 1].mode == Mode::Drive
                    && map.get(path[j].x, path[j].y) == Belief::Free
            });
            let mut repaired = false;
            if let (Some(j), Mode::Drive) = (rejoin, cur.mode) {
                let r = path[j];
                let p = cfg.repair_pad;
                let win = (
                    cur.x.min(r.x).saturating_sub(p),
                    cur.y.min(r.y).saturating_sub(p),
                    (cur.x.max(r.x) + p).min(map.w - 1),
                    (cur.y.max(r.y) + p).min(map.h - 1),
                );
                let lreq = PlanRequest { start: cur, goal: (r.x, r.y), drive_only: true, window: Some(win) };
                if let Some((local, _)) = plan(&map, &cfg.costs, &lreq) {
                    let replaced = j - i + 1;
                    let mut np = path[..i].to_vec();
                    np.extend_from_slice(&local);
                    np.extend_from_slice(&path[j + 1..]);
                    events.push(MissionEvent::LocalRepair {
                        step: trail.len() - 1,
                        detour_len: local.len().saturating_sub(replaced),
                        rejoined_at: (r.x, r.y),
                    });
                    path = np;
                    local_repairs += 1;
                    repaired = true;
                }
            }
            if !repaired {
                let greq = PlanRequest { start: cur, goal: world.goal, drive_only: false, window: None };
                match plan(&map, &cfg.costs, &greq) {
                    Some((np, cost)) => {
                        let mut full = path[..i].to_vec();
                        full.extend_from_slice(&np);
                        path = full;
                        global_replans += 1;
                        events.push(MissionEvent::GlobalReplan { step: trail.len() - 1, cost });
                    }
                    None => {
                        events.push(MissionEvent::Aborted { step: trail.len() - 1, why: "goal unreachable".into() });
                        break;
                    }
                }
            }
            continue; // re-validate the new path before moving
        }

        if i + 1 == path.len() {
            events.push(MissionEvent::Arrived { step: trail.len() - 1 });
            break;
        }

        let next = path[i + 1];
        t += 100;
        for s in SOURCES {
            sup.watchdog.heartbeat(s, t);
        }

        if next.mode != cur.mode {
            // takeoff / landing goes through the safety interlocks
            sup.interlocks.at_rest = true;
            sup.interlocks.landing_zone_clear = map.get(cur.x, cur.y) == Belief::Free;
            let ok = sup.handle(Event::RequestMode(next.mode), t).is_ok();
            if ok {
                t += 2_000; // takeoff / landing time
                for s in SOURCES {
                    sup.watchdog.heartbeat(s, t);
                }
                sup.tick(t);
                let _ = sup.handle(Event::TransitionComplete, t);
            }
            sup_log(sup, &mut events);
            if !ok {
                events.push(MissionEvent::Aborted { step: trail.len() - 1, why: "supervisor refused mode switch".into() });
                break;
            }
        } else {
            sup.interlocks.at_rest = false;
            sup.tick(t);
            sup_log(sup, &mut events);
        }
        if matches!(sup.state(), State::FailSafe { .. }) {
            events.push(MissionEvent::Aborted { step: trail.len() - 1, why: "supervisor fail-safe".into() });
            break;
        }

        executed_cost += step_cost(&map, &cfg.costs, cur, next);
        trail.push(next);
        i += 1;
    }

    let arrived = events.iter().any(|e| matches!(e, MissionEvent::Arrived { .. }));
    MissionReport {
        draft, final_map: map, global_plan: global, trail, events,
        planned_cost, executed_cost, local_repairs, global_replans, arrived,
    }
}

// ------------------------------------------------------------------ rendering

pub struct Paint {
    pub color: bool,
}

impl Paint {
    fn c(&self, code: &str, ch: char) -> String {
        if self.color {
            format!("\x1b[{}m{}\x1b[0m", code, ch)
        } else {
            ch.to_string()
        }
    }

    fn belief_char(&self, b: Belief) -> String {
        match b {
            Belief::Free => self.c("2", '.'),
            Belief::Blocked => self.c("90;1", '#'),
            Belief::Unknown => self.c("33", '?'),
        }
    }

    pub fn truth(&self, w: &World) -> String {
        let mut s = String::new();
        for y in 0..w.h {
            for x in 0..w.w {
                let ch = if (x, y) == w.start {
                    self.c("1", 'S')
                } else if (x, y) == w.goal {
                    self.c("1", 'G')
                } else {
                    match (w.terrain(x, y), w.canopy(x, y)) {
                        (Terrain::Rock, _) => self.c("31;1", 'o'),
                        (Terrain::Wall, true) => self.c("31;1", 'X'),
                        (Terrain::Wall, false) => self.c("90;1", '#'),
                        (Terrain::Water, _) => self.c("34;1", '~'),
                        (Terrain::Mud, _) => self.c("33", ','),
                        (Terrain::Free, true) => self.c("32", '%'),
                        (Terrain::Free, false) => self.c("2", '.'),
                    }
                };
                s.push_str(&ch);
            }
            s.push('\n');
        }
        s
    }

    /// Belief map with an optional path overlay.
    pub fn belief(&self, map: &Map, path: &[Step], start: (usize, usize), goal: (usize, usize), show_corrections: bool) -> String {
        let mut overlay: Vec<Option<char>> = vec![None; map.w * map.h];
        for (k, s) in path.iter().enumerate() {
            let switching = k > 0 && path[k - 1].mode != s.mode;
            let ch = if switching {
                if s.mode == Mode::Fly { 'T' } else { 'L' }
            } else if s.mode == Mode::Fly {
                '^'
            } else {
                '='
            };
            let i = s.y * map.w + s.x;
            if overlay[i] != Some('T') && overlay[i] != Some('L') {
                overlay[i] = Some(ch);
            }
        }
        let mut s = String::new();
        for y in 0..map.h {
            for x in 0..map.w {
                let i = y * map.w + x;
                let out = if (x, y) == start {
                    self.c("1", 'S')
                } else if (x, y) == goal {
                    self.c("1", 'G')
                } else if let Some(ch) = overlay[i] {
                    match ch {
                        '^' => self.c("35;1", '^'),
                        'T' | 'L' => self.c("35;1;7", ch),
                        _ => self.c("36;1", '='),
                    }
                } else if show_corrections && map.was_corrected(x, y) && map.get(x, y) == Belief::Blocked {
                    self.c("31;1", '!')
                } else {
                    self.belief_char(map.get(x, y))
                };
                s.push_str(&out);
            }
            s.push('\n');
        }
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::supervisor::{Config, Control};

    #[test]
    fn aerial_draft_misses_mud_rock_and_canopy() {
        let w = World::demo();
        let m = Map::from_survey(&w);
        assert_eq!(m.get(7, 8), Belief::Free); // mud looks fine from above
        assert_eq!(m.get(16, 8), Belief::Free); // rock is sub-resolution
        assert_eq!(m.get(35, 8), Belief::Unknown); // canopy hides the ground
        assert_eq!(m.get(26, 8), Belief::Blocked); // river is visible
    }

    #[test]
    fn river_forces_a_fly_segment() {
        let w = World::demo();
        let m = Map::from_survey(&w);
        let c = Costs::default();
        let start = Step { x: w.start.0, y: w.start.1, mode: Mode::Drive };
        let drive_only = PlanRequest { start, goal: w.goal, drive_only: true, window: None };
        assert!(plan(&m, &c, &drive_only).is_none());
        let hybrid = PlanRequest { start, goal: w.goal, drive_only: false, window: None };
        let (path, _) = plan(&m, &c, &hybrid).unwrap();
        let segs = segments(&path);
        assert!(segs.iter().any(|(m, _, _)| *m == Mode::Fly));
        assert_eq!(path.last().unwrap().mode, Mode::Drive);
    }

    #[test]
    fn mission_corrects_map_repairs_locally_and_arrives() {
        let w = World::demo();
        let mut sup = Supervisor::new(Config::default());
        let r = run_mission(&w, &mut sup, &MissionConfig::default());
        assert!(r.arrived, "{:?}", r.events);
        assert!(r.local_repairs >= 2);
        // the executed trail never drives onto ground-truth hazards
        for s in &r.trail {
            if s.mode == Mode::Drive {
                assert!(w.terrain(s.x, s.y).drivable(), "drove onto {:?} at {:?}", w.terrain(s.x, s.y), s);
            }
        }
        assert_eq!(*sup.state(), State::Armed { mode: Mode::Drive, control: Control::Auto });
    }
}
