use std::f64;
use std::fs::File;
use std::io::{self, Write};

use guidance::Target;
use terminal_size::{terminal_size, Width};
use yapb::Progress;

const WIDTH: usize = 1920;
const HEIGHT: usize = 1080;
const FRAMERATE: usize = 30;
const STEPS_PER_FRAME: usize = 10;
const TIMESTEP: f64 = 1.0 / (STEPS_PER_FRAME * FRAMERATE) as f64;

fn main() {
    let scenes = [Scene {
        missile: Body {
            position: na::Point3::new(0.0, 0.0, 0.0),
            velocity: na::Vector3::new(0., 1e3, 0.),
        },
        target: Body {
            position: na::Point3::new(1e4, 3e3, 0.0),
            velocity: na::Vector3::new(-2e3, 0., 0.),
        },
        max_steering_accel: 1e3,
    }];
    for (scene_num, scene) in scenes.iter().enumerate() {
        let mut scene = Sim::new(scene);
        let mut render = Render::new();
        let mut steps = 0;
        let mut progress = yapb::Bar::new();
        let max_distance = na::distance(&scene.target.position, &scene.missile.position);
        for i in 0.. {
            let rendered = i % STEPS_PER_FRAME == 0;
            if rendered {
                render.draw(scene_num, i / STEPS_PER_FRAME, &scene);
            }
            let done = scene.step();
            steps += 1;
            if let Some((Width(w), _)) = terminal_size() {
                let distance = na::distance(&scene.target.position, &scene.missile.position);
                progress.set(1. - (distance / max_distance) as f32);
                print!("\r{:width$}", progress, width = w as usize);
                io::stdout().flush().unwrap();
            }
            if done {
                break;
            }
        }
        let miss = na::distance(&scene.target.position, &scene.missile.position);
        if let Some((Width(w), _)) = terminal_size() {
            print!("\r{}\r", " ".repeat(w as usize));
        }
        println!("{}: {} steps; miss by {}; peak steering accel {}", scene_num, steps, miss, scene.peak_steering.norm());
    }
}

struct Render {
    missile_log: Vec<na::Point2<f64>>,
    target_log: Vec<na::Point2<f64>>,
}

impl Render {
    fn new() -> Self {
        Self {
            missile_log: Vec::new(),
            target_log: Vec::new(),
        }
    }

    fn draw(&mut self, scene_num: usize, frame_num: usize, scene: &Sim) {
        use cairo::*;

        let surface = ImageSurface::create(cairo::Format::Rgb24, WIDTH as i32, HEIGHT as i32).unwrap();
        let cr = Context::new(&surface);
        cr.translate(0., HEIGHT as f64);
        cr.scale(1.0, -1.0);
        cr.scale(0.2, 0.2);

        self.draw_body(&cr, (1.0, 0.5, 0.5), &scene.missile, &self.missile_log);

        self.draw_body(&cr, (0.5, 0.5, 1.0), &scene.target, &self.target_log);

        let mut file = File::create(format!("scene_{}-frame_{}.png", scene_num, frame_num)).unwrap();
        surface.write_to_png(&mut file).unwrap();

        self.missile_log.push(scene.missile.position.xy());
        self.target_log.push(scene.target.position.xy());
    }

    fn draw_body(&self, cr: &cairo::Context, rgb: (f64, f64, f64), body: &Body, trail: &[na::Point2<f64>]) {
        cr.save();
        let pixels = cr.device_to_user_distance(1., 0.).0;
        cr.set_line_width(2. * pixels);

        cr.set_source_rgb(rgb.0 * 0.5, rgb.1 * 0.5, rgb.2 * 0.5);
        if let Some(p) = trail.first() {
            cr.move_to(p.x, p.y);
            for p in trail.iter().skip(1) {
                cr.line_to(p.x, p.y);
            }
            cr.line_to(body.position.x, body.position.y);
            cr.stroke();
        }

        cr.set_source_rgb(rgb.0, rgb.1, rgb.2);
        cr.translate(body.position.x, body.position.y);
        cr.arc(0., 0., 6. * pixels, 0., 2. * f64::consts::PI);
        cr.fill();
        cr.move_to(0., 0.);
        let line = body.velocity * 0.05 * pixels;
        cr.line_to(line.x, line.y);
        cr.stroke();

        cr.restore();
    }
}

#[derive(Debug, Copy, Clone)]
struct Scene {
    missile: Body,
    target: Body,
    max_steering_accel: f64,
}

struct Sim {
    missile: Body,
    target: Body,
    max_steering_accel: f64,
    steps: u64,
    peak_steering: na::Vector3<f64>,
}

impl Sim {
    fn new(scene: &Scene) -> Self {
        Self {
            missile: scene.missile,
            target: scene.target,
            max_steering_accel: scene.max_steering_accel,
            steps: 0,
            peak_steering: na::zero(),
        }
    }

    fn step(&mut self) -> bool {
        const BOOST_TIME: f64 = 2.0;
        const MAX_BOOST: f64 = 1e3;
        const BOOST_ACCEL: f64 = MAX_BOOST / BOOST_TIME;

        let target = Target {
            position: self.target.position - self.missile.position,
            velocity: self.target.velocity - self.missile.velocity,
        };
        let steering = if target.is_closing() {
            let a = guidance::linear_steer(&target, &self.missile.velocity, self.missile.velocity.norm()).unwrap().0 / TIMESTEP;
            let ratio = self.max_steering_accel / a.norm();
            if ratio < 1.0 {
                a * ratio
            } else {
                a
            }
        } else {
            na::zero()
        };
        if steering.norm_squared() > self.peak_steering.norm_squared() {
            self.peak_steering = steering;
        }

        let boost = if self.steps as f64 * TIMESTEP > BOOST_TIME {
            na::zero()
        } else {
            self.missile.velocity.try_normalize(1e-3).unwrap_or_else(na::Vector3::y) * BOOST_ACCEL
        };

        self.target.integrate(na::zero());
        self.missile.integrate(steering + boost);
        self.steps += 1;
        !target.is_closing()
    }
}

#[derive(Debug, Copy, Clone)]
struct Body {
    position: na::Point3<f64>,
    velocity: na::Vector3<f64>,
}

impl Body {
    fn integrate(&mut self, acceleration: na::Vector3<f64>) {
        self.velocity += TIMESTEP * acceleration;
        self.position += TIMESTEP * self.velocity;
    }
}
