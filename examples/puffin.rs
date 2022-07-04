use lively_tk_lib::lively_tk::Solver;
use lively_tk_lib::objectives::core::matching::PositionMatchObjective;
use lively_tk_lib::objectives::core::base::CollisionAvoidanceObjective;
use lively_tk_lib::objectives::core::base::SmoothnessMacroObjective;
use lively_tk_lib::objectives::objective::Objective;
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

fn solver_function() {
  
    let data = fs::read_to_string("./tests/ur3e.xml")
    .expect("Something went wrong reading the file");
  
    let pos_match_obj = PositionMatchObjective::new("EE Position".to_string() , 20.0, "wrist_3_link".to_string());
    let col_avoid_obj = CollisionAvoidanceObjective::new("Collision Avoidance".to_string(), 10.0);
    let smooth_macro_obj = SmoothnessMacroObjective::new("Smoothness".to_string(), 10.0);

    let objective_vec : Vec<Objective> = vec![lively_tk_lib::objectives::objective::Objective::PositionMatch(pos_match_obj),
    lively_tk_lib::objectives::objective::Objective::CollisionAvoidance(col_avoid_obj),
    lively_tk_lib::objectives::objective::Objective::SmoothnessMacro(smooth_macro_obj)];
   
    let temp = Solver::new(data, objective_vec,None,None,None,None,None,None);
}



