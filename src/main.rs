use nannou::prelude::*;
use physics_engine::physics::{
    bounding_box::{BoundingBox, HasBoundingBox},
    collision_detection::PointCollision,
    engine::{Entity, PhysicsEngine, PhysicsEngineConfig},
    fixed_body::FixedBody,
    soft_body::{MassPoint, SoftBody, Spring},
};

fn main() {
    nannou::app(model).update(update).run();
}

struct Model {
    // Store the window ID so we can refer to this specific window later if needed.
    _window: WindowId,
    engine: PhysicsEngine,
    mouse_spring: Spring,
    attached_mass_point: Option<(usize, usize)>,
    framerate: usize,
}

fn model(app: &App) -> Model {
    // Create a new window! Store the ID so we can refer to it later.
    let _window = app
        .new_window()
        .size(1424, 1224)
        .title("nannou")
        .view(view) // The function that will be called for presenting graphics to a frame.
        .event(event) // The function that will be called when the window receives events.
        .build()
        .unwrap();

    let mut engine = PhysicsEngine::new(PhysicsEngineConfig {
        gravity: Vec2::new(0.0, -30.0),
        drag_coefficient: 0.02,
    });

    let mass = 0.3;
    let stiffness = 200.0;
    let damping = 2.0;
    let mass_point_dist = 10.0;
    let repel_force = 100.0;

    let make_body = |x, y, w, h| {
        Entity::Soft(SoftBody::new_rect(
            Vec2::new(x, y),
            Vec2::new(w, h),
            mass,
            stiffness,
            damping,
            repel_force,
            mass_point_dist,
        ))
    };

    engine
        .world_mut()
        // .add_entity(Entity::Fixed(FixedBody::Polygon {
        //     verts: vec![
        //         Vec2::new(-120.0, -50.0),
        //         Vec2::new(-100.0, -100.0),
        //         Vec2::new(100.0, -100.0),
        //         Vec2::new(120.0, -50.0),
        //         Vec2::new(140.0, -50.0),
        //         Vec2::new(120.0, -120.0),
        //         Vec2::new(-120.0, -120.0),
        //         Vec2::new(-140.0, -50.0),
        //     ],
        // }))
        // .add_entity(Entity::Fixed(FixedBody::Polygon {
        //     verts: vec![
        //         Vec2::new(-480.0, 100.0),
        //         Vec2::new(-480.0, 80.0),
        //         Vec2::new(-180.0, -100.0),
        //         Vec2::new(-180.0, -80.0),
        //     ],
        // }))
        .add_entity(Entity::Fixed(FixedBody::Polygon {
            verts: vec![
                Vec2::new(-240.0, -100.0),
                Vec2::new(-350.0, -400.0),
                Vec2::new(-400.0, -350.0),
            ],
        }))
        .add_entity(Entity::Fixed(FixedBody::Polygon {
            verts: vec![
                Vec2::new(350.0, 300.0),
                Vec2::new(340.0, 300.0),
                Vec2::new(360.0, 240.0),
                Vec2::new(370.0, 240.0),
            ],
        }))
        .add_entity(Entity::Fixed(FixedBody::Polygon {
            verts: vec![
                Vec2::new(450.0, 300.0),
                Vec2::new(460.0, 300.0),
                Vec2::new(440.0, 240.0),
                Vec2::new(430.0, 240.0),
            ],
        }))
        .add_entity(Entity::Fixed(FixedBody::Rect {
            pos: Vec2::new(-512.0, -512.0),
            size: Vec2::new(1024.0, 30.0),
        }))
        .add_entity(Entity::Fixed(FixedBody::Rect {
            pos: Vec2::new(-512.0, -472.0),
            size: Vec2::new(30.0, 1024.0),
        }))
        .add_entity(Entity::Fixed(FixedBody::Rect {
            pos: Vec2::new(482.0, -472.0),
            size: Vec2::new(30.0, 1024.0),
        }))
        .add_entity(Entity::Fixed(FixedBody::Polygon {
            verts: vec![
                Vec2::new(512.0, 562.0),
                Vec2::new(482.0, 562.0),
                Vec2::new(300.0, 900.0),
                Vec2::new(330.0, 900.0),
            ],
        }));

    let num_bodies_x = 10;
    let num_bodies_y = 2;
    let w = 50.0;
    let h = 50.0;

    for x in -num_bodies_x / 2..num_bodies_x / 2 + 1 {
        for y in 0..num_bodies_y {
            engine.world_mut().add_entity(make_body(
                x as f32 * w * 1.1,
                100.0 + y as f32 * h * 2.0,
                w,
                h,
            ));
        }
    }

    Model {
        _window,
        engine,
        mouse_spring: Spring::new(0.0, 100.0, 50.0),
        attached_mass_point: None,
        framerate: 60,
    }
}

fn update(app: &App, model: &mut Model, update: Update) {
    const ITERATIONS: usize = 50;

    for _ in 0..ITERATIONS {
        if let Some((i, j)) = model.attached_mass_point {
            let mass = &mut model.engine.world_mut().soft_bodies_mut()[i].points_mut()[j];
            let mouse_mass = MassPoint::new(100.0, app.mouse.position());
            let force = model.mouse_spring.calculate_force(&mass, &mouse_mass);
            mass.add_force(force);
        }
        model
            .engine
            .update(3.0 * update.since_last.as_secs_f32() / (ITERATIONS as f32));
    }
    model.framerate = (1.0 / update.since_last.as_secs_f64()) as usize;
}

// Handle events related to the window and update the model if necessary
fn event(app: &App, model: &mut Model, event: WindowEvent) {
    match event {
        MousePressed(button) => match button {
            MouseButton::Left => {
                let mut pair = None;
                let mut best_dist = f32::MAX;

                for (i, soft_body) in model.engine.world().soft_bodies().iter().enumerate() {
                    if soft_body
                        .get_bounding_box()
                        .contains_point(app.mouse.position())
                    {
                        for (j, mass) in soft_body.points().iter().enumerate() {
                            let dist = mass.pos().distance(app.mouse.position());
                            if dist < 50.0 && dist < best_dist {
                                best_dist = dist;
                                pair = Some((i, j));
                            }
                        }
                    }
                }

                model.attached_mass_point = pair;
            }
            _ => {}
        },
        MouseReleased(button) => match button {
            MouseButton::Left => model.attached_mass_point = None,
            _ => {}
        },
        _ => {}
    }
}

// Draw the state of your `Model` into the given `Frame` here.
fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(WHITE);

    for fixed_body in model.engine.world().fixed_bodies().iter() {
        fixed_body.draw(&draw);
        // fixed_body.draw_closest_point(&draw, app.mouse.position());
    }

    for soft_body in model.engine.world().soft_bodies().iter() {
        soft_body.draw(&draw);
    }

    if let Some((i, j)) = model.attached_mass_point {
        let mass = &model.engine.world().soft_bodies()[i].points()[j];
        draw.line()
            .start(app.mouse.position())
            .end(mass.pos())
            .color(RED);
    }

    draw.text(&format!("Framerate: {} fps", model.framerate))
        .x(0.0)
        .y(460.0)
        .color(BLACK);

    draw.to_frame(app, &frame).unwrap();
}

trait Drawable {
    fn draw(&self, draw: &Draw);
}

trait CollisionDrawable {
    fn draw_closest_point(&self, draw: &Draw, mouse: Vec2);
}

impl Drawable for FixedBody {
    fn draw(&self, draw: &Draw) {
        match self {
            Self::Circle { center, radius } => {
                draw.ellipse().xy(*center).radius(*radius).color(BLACK);
            }
            FixedBody::Polygon { verts } => {
                draw.polygon().points(verts.iter().map(|v| *v)).color(BLACK);
            }
            FixedBody::Rect { pos, size } => {
                draw.rect().xy(*pos + *size / 2.0).wh(*size).color(BLACK);
            }
        }
        // self.get_bounding_box().draw(draw);
    }
}

impl Drawable for SoftBody {
    fn draw(&self, draw: &Draw) {
        for mass in self.points().iter() {
            draw.ellipse().xy(mass.pos()).radius(5.0).color(BLUE);
        }
        for (a, b, _) in self.connections() {
            draw.line()
                .start(self.points()[*a].pos())
                .end(self.points()[*b].pos())
                .color(BLUE);
        }
        self.get_bounding_box().draw(draw);
    }
}

impl CollisionDrawable for FixedBody {
    fn draw_closest_point(&self, draw: &Draw, mouse: Vec2) {
        if let Some(p) = self.closest_point_on_edge(mouse) {
            draw.ellipse().xy(p).radius(4.0).color(RED);
        }
    }
}

impl Drawable for BoundingBox {
    fn draw(&self, draw: &Draw) {
        draw.rect()
            .no_fill()
            .stroke(GREEN)
            .stroke_weight(1.0)
            .xy((self.start() + self.stop()) / 2.0)
            .wh(self.stop() - self.start());
    }
}
