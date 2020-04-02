#include <multiview_visualiser.hpp>

#include <pangolin/display/image_view.h>

MultiviewVisualiser::MultiviewVisualiser(std::string window_name, size_t n_cam) : N_CAM(n_cam), 
    window_name_(window_name), camera_queues_(nullptr), visualisation_queues_(nullptr)
    // ,but_next("ui.next_point"), bu
{

}

// void MultiviewVisualiser::addUIPanel()
// {

// }

void MultiviewVisualiser::run()
{
    if (!camera_queues_ || !visualisation_queues_)
    {
        std::cout << "run(): queues not initialised..." << std::endl;
        return; 
    }
        
    pangolin::CreateWindowAndBind(window_name_ , 1600, 900);

    // enable spatial occlusion awareness in OpenGL
    glEnable(GL_DEPTH_TEST);

    // addUIPanel();

    // Create views for video stream of cameras
    // pangolin::Attach bottom, pangolin::Attach top, pangolin::Attach left, pangolin::Attach right 
    auto& img_view_display =
        pangolin::CreateDisplay()
            .SetBounds(0.2, 1.0, 0.0, 0.4)
            .SetLayout(pangolin::LayoutEqual);

    img_view_display_ptr = &img_view_display;

    // Add stereo images
    std::vector<std::shared_ptr<pangolin::ImageView>> img_view;

    for (size_t vid = 0; vid < 2; vid++)
    {
        auto view = img_view.emplace_back(std::make_shared<pangolin::ImageView>());
        img_view_display.AddDisplay(*view);
        view->SetDrawFunction(std::bind(&MultiviewVisualiser::drawImageViewOverlay, this, std::placeholders::_1, vid));
        view->OnSelectionCallback = std::bind(&MultiviewVisualiser::OnSelectionCallback, this, std::placeholders::_1, vid);
    }

    // Transparency
    glEnable(GL_BLEND);
    // https://gamedev.stackexchange.com/questions/82741/normal-blend-mode-with-opengl-trouble
    // https://learnopengl.com/Advanced-OpenGL/Blending
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    

    while (!pangolin::ShouldQuit() ) {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if(visualisation_queues_->try_pop(latest_synced_vis_data_)){
            std::cout << "drawing visualisation points and lines" << std::endl;
        }

        // Obtain images
        img_view_display.Activate();

        //// Process Incoming Synced Camera Stream
        if (camera_queues_->try_pop(latest_synced_images_))
        {
            pangolin::GlPixFormat fmt;
            fmt.glformat = GL_LUMINANCE;
            fmt.gltype = GL_UNSIGNED_BYTE;
            fmt.scalable_internal_format = GL_LUMINANCE8;

            for (int i = 0; i < 2; i++)
            {
                auto& image = latest_synced_images_->images[i];
                img_view[i]->SetImage(image.ptr, image.w, image.h, image.pitch, fmt);
            }
        }

        pangolin::FinishFrame();

    }

    

    ui_data_queues_.emplace(); // push empty to signal end
}

void MultiviewVisualiser::OnSelectionCallback(pangolin::ImageViewHandler::OnSelectionEventData e, size_t cam_id)
{
    if (!e.dragging){
        auto data = std::make_shared<SyncedUIData>();
        float x,y;
        e.handler.GetHover(x, y);
        auto& pt =data->points_2d.emplace_back();
        pt.xy = {x,y};
        pt.view_id = cam_id;
        
        // visuals
        pt.point_size = 2;
        pt.colour = {200,0,200};

        ui_data_queues_.push(data);
        // std::cout << ui_data_queues_.size()  << " OnSelectionCallback = "<< &ui_data_queues_ << std::endl;
    }
    
}

// This function is run by Pangolin automatically for all viewports, set by SetDrawFunction()
void MultiviewVisualiser::drawImageViewOverlay(pangolin::View& v, size_t cam_id)
{
    v.Activate();

    // Execute if visualisation data is available
    if (latest_synced_vis_data_.get()){
        // glLineWidth(1.0);
        // glColor3f(1.0, 0.0, 0.0);
        // glEnable(GL_BLEND);
        // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        //// Draw all points
        for (auto& pt : latest_synced_vis_data_->points_2d[cam_id]){
            // std::cout << cam_id << ": " << pt.xy.transpose() << std::endl;
            glColor3fv(pt.colour.data()); // float colour
            pangolin::glDrawCircle(pt.xy, pt.point_size);
        }

        //// Draw all lines
        for(auto& line : latest_synced_vis_data_->lines_2d[cam_id]){
            glColor3fv(line.begin.colour.data());
            glPointSize(line.line_size);
            pangolin::glDrawLines(std::vector<Eigen::Vector2d>{line.begin.xy, line.end.xy});
        }
        
        
    }

}