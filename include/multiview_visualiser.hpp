#pragma once

#include <pangolin/pangolin.h>
#include <basalt/image/image_pyr.h>

#include <tbb/tbb.h>

#include <common_struct.hpp>

#include <pangolin/handler/handler_image.h>

struct SyncedImages{
    using Ptr = std::shared_ptr<SyncedImages>;

    uint64_t time_ns;
    std::vector<basalt::ManagedImage<uint8_t>> images;
};

struct SyncedVisualisationData{
    using Ptr = std::shared_ptr<SyncedVisualisationData>;
    
    uint64_t time_ns;
    const size_t N;

    // 2D data
    std::vector<Point2D::List> points_2d;
    std::vector<Line2D::List> lines_2d;
    // TODO: trajectories

    // 3D data
    std::vector<Point3D> points_3d_nwu; // should be in world coordinates (3, N_POINTS)
    std::vector<Sophus::SE3d> camera_poses;

    SyncedVisualisationData() = delete;
    SyncedVisualisationData(size_t n_cam) : N(n_cam) {
        points_2d.resize(N); 
        lines_2d.resize(N);
        points_3d_nwu.resize(N);
        camera_poses.resize(N);
    };
};

struct SyncedUIData{
    using Ptr = std::shared_ptr<SyncedUIData>;

    std::vector<Point2D> points_2d;
};


class MultiviewVisualiser{
public:

    const size_t N_CAM;
    

    MultiviewVisualiser() = delete;
    MultiviewVisualiser(std::string window_name, size_t n_cam);

    void connectCameraQueue(tbb::concurrent_bounded_queue<SyncedImages::Ptr>& q){camera_queues_ = &q;};
    void connectPointLineQueue(tbb::concurrent_bounded_queue<SyncedVisualisationData::Ptr>& q){visualisation_queues_ = &q;};
    
    // void addUIPanel();
    void run();

    void printAddress(){std::cout << "print address ui_data_queues_ " << &ui_data_queues_ << std::endl;};

    tbb::concurrent_bounded_queue<SyncedUIData::Ptr> ui_data_queues_; // outputs

private:
    std::string window_name_;
    tbb::concurrent_bounded_queue<SyncedImages::Ptr>* camera_queues_;
    tbb::concurrent_bounded_queue<SyncedVisualisationData::Ptr>* visualisation_queues_;
    
    

    SyncedImages::Ptr latest_synced_images_ = nullptr;
    SyncedVisualisationData::Ptr latest_synced_vis_data_ = nullptr;

    //UI
    // using Button = pangolin::Var<std::function<void(void)>>;
    // Button but_next, but_remove;

    pangolin::View* img_view_display_ptr;

    void OnSelectionCallback(pangolin::ImageViewHandler::OnSelectionEventData, size_t cam_id);

    void drawImageViewOverlay(pangolin::View& v, size_t cam_id);
};