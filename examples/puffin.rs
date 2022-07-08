use lively_tk_lib::lively_tk::Solver;
use lively_tk_lib::objectives::core::matching::PositionMatchObjective;
use lively_tk_lib::objectives::core::base::CollisionAvoidanceObjective;
use lively_tk_lib::objectives::core::base::SmoothnessMacroObjective;
use lively_tk_lib::objectives::objective::Objective;
use lively_tk_lib::utils::goals::Goal::ScalarRange;
use lively_tk_lib::utils::shapes::BoxShape;
use nalgebra::geometry::Isometry3;

use std::fs;

mod imgui_support;
use imgui_support::ImguiManager;

mod renderer;
use renderer::Renderer;
use puffin::set_scopes_on;

fn update() {
    // profiling::scope!("update");
    
    solver_function();
    
}

fn draw(
    imgui_manager: &ImguiManager,
    profiler_ui: &mut puffin_imgui::ProfilerUi,
) {
    profiling::scope!("draw");
    //
    //Draw an inspect window for the example struct
    //
    imgui_manager.with_ui(|ui| {
        profiler_ui.window(ui);
    });
}

fn burn_time(micros: u128) {
    let start_time = std::time::Instant::now();
    loop {
        if (std::time::Instant::now() - start_time).as_micros() > micros {
            break;
        }
    }
}

fn main() {
    // Setup logging
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Debug)
        .init();

    // Enable puffin
    puffin::set_scopes_on(true);

    // Create the winit event loop
    let event_loop = winit::event_loop::EventLoop::<()>::with_user_event();

    // Set up the coordinate system to be fixed at 900x600, and use this as the default window size
    // This means the drawing code can be written as though the window is always 900x600. The
    // output will be automatically scaled so that it's always visible.
    let logical_size = winit::dpi::LogicalSize::new(900.0, 600.0);

    // Create a single window
    let window = winit::window::WindowBuilder::new()
        .with_title("Profiling Demo")
        .with_inner_size(logical_size)
        .build(&event_loop)
        .expect("Failed to create window");

    // Initialize imgui
    let imgui_manager = imgui_support::init_imgui_manager(&window);

    let renderer = Renderer::new(&window, imgui_manager.font_atlas_texture());

    // Check if there were errors setting up vulkan
    if let Err(e) = renderer {
        println!("Error during renderer construction: {:?}", e);
        return;
    }

    let mut renderer = renderer.unwrap();

    let mut profiler_ui = puffin_imgui::ProfilerUi::default();

    // Start the window event loop. Winit will not return once run is called. We will get notified
    // when important events happen.
    event_loop.run(move |event, _window_target, control_flow| {
        imgui_manager.handle_event(&window, &event);

        match event {
            //
            // Halt if the user requests to close the window
            //
            winit::event::Event::WindowEvent {
                event: winit::event::WindowEvent::CloseRequested,
                ..
            } => *control_flow = winit::event_loop::ControlFlow::Exit,

            //
            // Close if the escape key is hit
            //
            winit::event::Event::WindowEvent {
                event:
                    winit::event::WindowEvent::KeyboardInput {
                        input:
                            winit::event::KeyboardInput {
                                virtual_keycode: Some(winit::event::VirtualKeyCode::Escape),
                                ..
                            },
                        ..
                    },
                ..
            } => *control_flow = winit::event_loop::ControlFlow::Exit,

            //
            // Request a redraw any time we finish processing events
            //
            winit::event::Event::MainEventsCleared => {
                update();

                // Queue a RedrawRequested event.
                window.request_redraw();
            }

            //
            // Redraw
            //
            winit::event::Event::RedrawRequested(_window_id) => {
                imgui_manager.begin_frame(&window);
                draw(&imgui_manager, &mut profiler_ui);
                imgui_manager.render(&window);

                if let Err(e) = renderer.draw(&window, imgui_manager.draw_data()) {
                    println!("Error during draw: {:?}", e);
                    *control_flow = winit::event_loop::ControlFlow::Exit
                }

                profiling::finish_frame!();
            }

            //
            // Ignore all other events
            //
            _ => {}
        }
    });
}


// staticData, 
//     [],
//     [ScalarRange(value = 0, delta = 0), 
//      ScalarRange(value = -0.15, delta = 0), 
//      ScalarRange(value = 0, delta = 0), 
//      ScalarRange(value = 0, delta = 0), 
//      ScalarRange(value = 0, delta = 0), 
//      ScalarRange(value = 0, delta = 0),
//      ],
//      [
//
//       BoxShape(name = "conveyorCollisionShapeBelt", frame="world", physical = True, x = 0.75, y = 0.65, z = 0.25, local_transform = Transform(translation = Translation( 0.9499698,-0.2499698,0.050000000000000044), rotation = Rotation(0,0,-0.7069999677447772,0.7072135784958345))),
//       BoxShape(name = "conveyorRecieverCollisionShapeBase", frame="world", physical = True, x = 0.75, y = 0.25, z = 0.7, local_transform = Transform(translation = Translation( -0.5500906000000001,-0.25009060000000005, -0.45), rotation = Rotation(0,0,0.7069999677447771,0.7072135784958345))),
//       BoxShape(name = "conveyorRecieverCollisionShapeLeftSplit", frame="world", physical = True, x = 0.3, y = 0.155, z = 0.165, local_transform = Transform(translation = Translation(  -0.59013137,-0.42502567,-0.025000000000000022), rotation = Rotation(0,0,0.7069999677447772,0.7072135784958345))),
//       BoxShape(name = "conveyorRecieverCollisionShapeRightSplit", frame="world", physical = True, x = 0.3, y = 0.155, z = 0.165, local_transform = Transform(translation = Translation(-0.59002567,-0.07513137000000006,-0.025000000000000022), rotation = Rotation(0,0,0.7069999677447772,0.7072135784958345))),
//       BoxShape(name ="conveyorDispatcherCollisionShapeBase", frame="world", physical = True, x = 0.75, y = 0.35, z = 0.9, local_transform = Transform(translation = Translation(0.6000755,-0.2500755,-0.3), rotation = Rotation(0,0,-0.7069999677447771,0.7072135784958345))),
//       BoxShape(name ="conveyorDispatcherCollisionShapeLeftSplit", frame="world", physical = True, x = 0.255, y = 0.275, z = 0.175, local_transform = Transform(translation = Translation(0.65000755,-0.07511325000000005,0.22499999999999998), rotation = Rotation(0,0,-0.7069999677447771,0.7072135784958345))),
//       BoxShape(name ="conveyorDispatcherCollisionShapeRightSplit", frame="world", physical = True, x = 0.29, y = 0.275, z = 0.175, local_transform = Transform(translation = Translation(0.65011325,-0.42500755,0.22499999999999998), rotation = Rotation(0,0,-0.7069999677447771,0.7072135784958345))),
//       BoxShape(name ="tableCollisionShapeTop", frame="world", physical = True, x = 0.1225, y = 0.625, z = 0.05, local_transform = Transform(translation = Translation(0,0.36,-0.010000000000000009), rotation = Rotation(0,0,0,1))),
//       BoxShape(name ="tableCollisionShapeFrontLeftLeg", frame="world", physical = True, x = 0.05, y = 0.05, z = 0.75, local_transform = Transform(translation = Translation(-0.585,0.07,-0.395), rotation = Rotation(0,0,0,1))),
//       BoxShape(name ="tableCollisionShapeRearLeftLeg", frame="world", physical = True, x = 0.05, y = 0.05, z = 0.75, local_transform = Transform(translation = Translation(-0.585,0.6499999999999999,-0.585), rotation = Rotation(0,0,0,1))),
//       BoxShape(name ="tableCollisionShapeRearRightLeg", frame="world", physical = True, x = 0.05, y = 0.05, z = 0.75, local_transform = Transform(translation = Translation(0.585,0.7,-0.395), rotation = Rotation(0,0,0,1))), 
//       BoxShape(name ="pedestalCollisionShapeBase", frame="world", physical = True, x = 0.65, y = 0.65, z = 0.15, local_transform = Transform(translation = Translation(0,-0.15,-0.7150000000000001), rotation = Rotation(0,0,0,1))),
//       BoxShape(name ="pedestalCollisionShapeTower", frame="world", physical = True, x = 0.1, y = 0.1, z = 0.7, local_transform = Transform(translation = Translation(0,-0.15,-0.33), rotation = Rotation(0,0,0,1))),
//       BoxShape(name ="mk2CollisionShapeLeftVertical", frame="world", physical = True, x = 0.125, y = 0.185, z = 0.4, local_transform = Transform(translation = Translation(-0.46,0.42000000000000004,0.22500000000000003), rotation = Rotation(0,0,0,1))),
//       BoxShape(name ="mk2CollisionShapeRightVertical", frame="world", physical = True, x = 0.125, y = 0.225, z = 0.4, local_transform = Transform(translation = Translation(-0.10000000000000003,0.445,0.22500000000000003), rotation = Rotation(0,0,0,1))),
//       BoxShape(name ="mk2CollisionShapeBase", frame="world", physical = True, x = 0.4, y = 0.4, z = 0.1, local_transform = Transform(translation = Translation( -0.28,0.32,0.050000000000000044), rotation = Rotation(0,0,0,1))),
//       BoxShape(name ="mk2CollisionShapeSpool", frame="world", physical = True, x = 0.4, y = 0.25, z = 0.25, local_transform = Transform(translation = Translation( -0.28, 0.445,0.48000000000000004), rotation = Rotation(0,0,0,1))),  
//      ],

fn solver_function() {
  
    let data = fs::read_to_string("./tests/ur3e.xml")
    .expect("Something went wrong reading the file");
  
    let pos_match_obj = PositionMatchObjective::new("EE Position".to_string() , 20.0, "wrist_3_link".to_string());
    let col_avoid_obj = CollisionAvoidanceObjective::new("Collision Avoidance".to_string(), 10.0);
    let smooth_macro_obj = SmoothnessMacroObjective::new("Smoothness".to_string(), 10.0);
    let scalar_range_1 = ScalarRange{value : 0.0 , delta : 0.0};
    let scalar_range_2 = ScalarRange{value : -0.15, delta : 0.0};
    let scalar_range_3 = ScalarRange{value : 0.0 , delta : 0.0};
    let scalar_range_4 = ScalarRange{value : 0.0, delta : 0.0};
    let scalar_range_5 = ScalarRange{value : 0.0 , delta : 0.0};
    let scalar_range_6 = ScalarRange{value : 0.0 , delta : 0.0};
    let scalar_range_vec : Vec<lively_tk_lib::utils::goals::Goal> = vec![scalar_range_1,scalar_range_2,scalar_range_3,scalar_range_4,scalar_range_5,scalar_range_6];
    let iso_1 = Isometry3::new(vec![1.7497281999999998,-0.24972819999999987,0.050000000000000044], vec![0.0,0.0,-0.7069999677447771,0.7072135784958345]);
    let box_1 = BoxShape::new("conveyorCollisionShapeBase".to_string(), "world".to_string(), true, 1.0, 1.1, 1.7, iso_1 );

    let iso_2 = Isometry3::new(vec![ 0.9499698,-0.2499698,0.050000000000000044], vec![0.0,0.0,-0.7069999677447772,0.7072135784958345]);
    let box_2 = BoxShape::new("conveyorCollisionShapeBelt".to_string(), "world".to_string(), true, 0.75, 0.65, 0.25,iso_2 );

    let iso_3 = Isometry3::new(vec![ -0.5500906000000001,-0.25009060000000005, -0.45], vec![0.0,0.0,0.7069999677447771,0.7072135784958345]);
    let box_3 = BoxShape::new("conveyorRecieverCollisionShapeBase".to_string(), "world".to_string(), true, 0.75, 0.25, 0.7,iso_3 );

    let iso_4= Isometry3::new(vec![ -0.59013137,-0.42502567,-0.025000000000000022], vec![0.0,0.0,0.7069999677447772,0.7072135784958345]);
    let box_4 = BoxShape::new("conveyorRecieverCollisionShapeLeftSplit".to_string(), "world".to_string(), true, 0.3, 0.155, 0.165,iso_4 );

    let iso_5= Isometry3::new(vec![ -0.59002567,-0.07513137000000006,-0.025000000000000022], vec![0.0,0.0,0.7069999677447772,0.7072135784958345]);
    let box_5 = BoxShape::new("conveyorRecieverCollisionShapeRightSplit".to_string(), "world".to_string(), true, 0.3, 0.155, 0.165,iso_5 );

    let iso_6= Isometry3::new(vec![ 0.6000755,-0.2500755,-0.3], vec![0.0,0.0,-0.7069999677447771,0.7072135784958345]);
    let box_6 = BoxShape::new("conveyorDispatcherCollisionShapeBase".to_string(), "world".to_string(), true, 0.75, 0.35, 0.9,iso_6 );

    let iso_7= Isometry3::new(vec![0.65000755,-0.07511325000000005,0.22499999999999998], vec![0.0,0.0,-0.7069999677447771,0.7072135784958345]);
    let box_7 = BoxShape::new("conveyorDispatcherCollisionShapeLeftSplit".to_string(), "world".to_string(), true, 0.255, 0.275, 0.175,iso_7 );

    let iso_8= Isometry3::new(vec![0.65011325,-0.42500755,0.22499999999999998], vec![0.0,0.0,-0.7069999677447771,0.7072135784958345]);
    let box_8 = BoxShape::new("conveyorDispatcherCollisionShapeRightSplit".to_string(), "world".to_string(), true,0.29, 0.275, 0.175,iso_8 );

    let iso_9 = Isometry3::new(vec![0.65011325,-0.42500755,0.22499999999999998], vec![0.0,0.0,-0.7069999677447771,0.7072135784958345]);
    let box_9 = BoxShape::new("conveyorDispatcherCollisionShapeRightSplit".to_string(), "world".to_string(), true,0.29, 0.275, 0.175,iso_9 );

    let iso_10 = Isometry3::new(vec![0.0,0.36,-0.010000000000000009], vec![0.0,0.0,0.0,1.0]);
    let box_10 = BoxShape::new("tableCollisionShapeTop".to_string(), "world".to_string(), true, 0.1225, 0.625, 0.05,iso_10 );

    let iso_11 = Isometry3::new(vec![-0.585,0.07,-0.395], vec![0.0,0.0,0.0,1.0]);
    let box_11 = BoxShape::new("tableCollisionShapeFrontLeftLeg".to_string(), "world".to_string(), true, 0.05,0.05,0.75,iso_11 );

    let iso_12 = Isometry3::new(vec![-0.585,0.6499999999999999,-0.585], vec![0.0,0.0,0.0,1.0]);
    let box_12 = BoxShape::new("tableCollisionShapeRearLeftLeg".to_string(), "world".to_string(), true, 0.05, 0.05,0.75 ,iso_12 );

    let iso_12 = Isometry3::new(vec![-0.585,0.6499999999999999,-0.585], vec![0.0,0.0,0.0,1.0]);
    let box_12 = BoxShape::new("tableCollisionShapeRearLeftLeg".to_string(), "world".to_string(), true, 0.05, 0.05,0.75 ,iso_12 );









    


    
    let objective_vec : Vec<Objective> = vec![lively_tk_lib::objectives::objective::Objective::PositionMatch(pos_match_obj),
    lively_tk_lib::objectives::objective::Objective::CollisionAvoidance(col_avoid_obj),
    lively_tk_lib::objectives::objective::Objective::SmoothnessMacro(smooth_macro_obj)];
   
    let temp = Solver::new(data, objective_vec,None,None,None,None,None,None);

}



