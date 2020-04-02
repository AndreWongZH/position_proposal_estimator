#include <multiview_geometry.hpp>
#include <multiview_visualiser.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>

int main(){

    MultiViewGeometry mvg;
    // CalibrationParser calib_parser("../params/cam_calib_est.yaml");

    mvg.parseYamlConfig("../params/cam_calib_est.yaml");

    // MultiviewVisualiser vis("test vis", mvg.N_cam);
    // MultiviewVisualiser vis = new MultiviewVisualiser("test vis", mvg.N_cam);

    MultiviewVisualiser vis("test vis", mvg.N_cam);

    tbb::concurrent_bounded_queue<SyncedImages::Ptr> camera_queues;
    tbb::concurrent_bounded_queue<SyncedVisualisationData::Ptr> visualisation_queues;
    tbb::concurrent_bounded_queue<SyncedUIData::Ptr>* ui_data_queues; // input from ad-hoc user inputs

    vis.connectCameraQueue(camera_queues);
    vis.connectPointLineQueue(visualisation_queues);

    vis.printAddress();
    ui_data_queues = &(vis.ui_data_queues_);
    std::cout<< " main = " << &(*ui_data_queues)<< std::endl;
    
    

    // Load image
    cv::Mat imagel = cv::imread("../samples/00-video-left.png", cv::IMREAD_GRAYSCALE);
    cv::Mat imager = cv::imread("../samples/00-video-right.png", cv::IMREAD_GRAYSCALE);

    std::cout << imagel.step << std::endl;


    basalt::Image<uint8_t> bimagel(imagel.data , imagel.cols,imagel.rows, imagel.step);
    basalt::Image<uint8_t> bimager(imager.data, imager.cols,imager.rows, imager.step);

    auto synced_images = std::make_shared<SyncedImages>();
    synced_images->images.emplace_back().CopyFrom(bimagel); // create basalt::ManagedImage<uint8_t>()
    synced_images->images.emplace_back().CopyFrom(bimager);

    camera_queues.push(synced_images);


    // GUI runs in another thread
    std::thread t_gui(&MultiviewVisualiser::run, std::ref(vis)); // hm: Must have std::ref, to force std::thread to not to copy
    // vis.run();

    std::cout << ui_data_queues->size() << &(*ui_data_queues)<< std::endl;

    //// Set test points
    // auto synced_vis_data = std::make_shared<SyncedVisualisationData>(2);

    // auto& pt = synced_vis_data->points_2d[0].emplace_back();
    // pt.xy << 50, 120;
    // pt.point_size = 5;
    // pt.colour = {200,200,0};
    // visualisation_queues.push(synced_vis_data);

    // project all Epipoles
    {
        auto synced_vis_data = std::make_shared<SyncedVisualisationData>(mvg.N_cam);
        for (size_t i = 0 ; i < mvg.N_cam ; i++)
            for (size_t j = 0 ; j < mvg.N_cam ; j++){
                if (i == j) continue;
                auto& pt = synced_vis_data->points_2d[j].emplace_back();
                pt.xy = mvg.projectEpipole(i,j);
                pt.point_size = 5;
                pt.colour = {200,200,0};
            }
        
        visualisation_queues.push(synced_vis_data);
    }
    
    //// Process Mouse Clicks received from Pangolin UI

    SyncedUIData::Ptr ui_data;
    while(true){
        // std::cout << ui_data_queues->size() << &(*ui_data_queues)<< std::endl;
        
        if(ui_data_queues->try_pop(ui_data))
        {
            
            if (!ui_data.get()) break; // come to the end, quiting
            auto synced_vis_data = std::make_shared<SyncedVisualisationData>(mvg.N_cam);
            for (auto& pt : ui_data->points_2d){
                // add 2d point in current view
                auto& pt_curr_view = synced_vis_data->points_2d[pt.view_id].emplace_back();
                pt_curr_view.colour = {1,0,0};
                pt_curr_view.xy = pt.xy;

                // calculate epipolar line projections in other views
                for (size_t to_cam = 0; to_cam < mvg.N_cam; to_cam++){
                    if (to_cam == pt.view_id) continue;
                    Eigen::Vector3d line = mvg.project2EpipolarLineinPixel(pt.xy, pt.view_id, to_cam);
                    Eigen::Vector3d line_x_0, line_x_width;
                    line_x_0 << 1,0,0;
                    line_x_width << 1,0,-672;

                    auto res0 = line_x_0.cross(line);
                    auto reswidth = line_x_width.cross(line);

                    std::cout << (res0 / res0[2]).transpose()  << std::endl;
                    std::cout << (reswidth / reswidth[2]).transpose() << std::endl;
                    auto& pt_other = synced_vis_data->lines_2d[to_cam].emplace_back();
                    pt_other.begin.colour = pt_other.end.colour = {0,1,1};
                    pt_other.begin.xy << 0 , (res0 / res0[2])[1];
                    pt_other.end.xy << 672 , (reswidth / reswidth[2])[1];

                    std::cout << to_cam << ": "<< line.transpose() << std::endl;
                    
                    // pt_other.begin.xy = ;
                }
                
            }
            visualisation_queues.push(synced_vis_data);

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "main() terminating" << std::endl;

    t_gui.join();
    return 0;
}