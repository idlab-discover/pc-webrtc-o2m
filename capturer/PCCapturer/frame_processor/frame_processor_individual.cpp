#include "frame_processor_individual.h"
#include "../sampler/sampler_uniform.h"
#include "../encoder/encoder_draco.h"
#include "../worker/worker_layered.h"

FrameProcessorIndividual::FrameProcessorIndividual(std::shared_ptr<Camera> _cam, std::shared_ptr<Transporter> _transporter, uint8_t n_threads) : FrameProcessor(_cam, _transporter)
{
    tp.Init(n_threads, 3);
}

int FrameProcessorIndividual::ProcessNextFrame()
{
    std::unique_lock lk(m);
    std::shared_ptr<int> completed_jobs = std::make_shared<int>(0);
    curr_frame++;
    cam->captureFrame(true);

    Eigen::Vector4f min_p;
    Eigen::Vector4f max_p;
    pcl::getMinMax3D(*cam->getCurrentFrame(), min_p, max_p);
    pcl::PointXYZ min_pt(min_p[0], min_p[1], min_p[2]);
    pcl::PointXYZ max_pt(max_p[0], max_p[1], max_p[2]);
    std::map<uint64_t, uint32_t> client_to_bw = transporter->GetClientBitrates();
    uint64_t n_clients = client_to_bw.size();
    cloud_pointer curr_frame = this->cam->getCurrentFrame();
    int curr_frame_nr = this->curr_frame;
    std::chrono::milliseconds start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    );
    ResultWriter::setLayersPerFrame(curr_frame_nr, n_clients);
    for (auto const& [client_id, bw] : client_to_bw)
    {
      
        tp.AddJob([this, client_id, bw, completed_jobs, start_time, curr_frame, curr_frame_nr, n_clients, min_pt, max_pt] {
            float sample_rate = this->predictSampleRate(bw*0.9);
            // std::cout << i << " " <<  sample_rate << " " << compression_factor << std::endl;
            std::vector<float> layer_ratios = { sample_rate };
            SamplerUniform s(layer_ratios);
            // std::cout << i << " | " << curr_frame << std::endl;
            s.setFrameId(curr_frame_nr);
            s.setCloud(curr_frame);
            s.createLayers();

            EncoderDraco d;
            d.setLayerId(client_id);
            d.setCloud(s.getLayers()[0]);
            d.setFrameId(curr_frame_nr);
            const char* encoded_data = d.encode();

            std::vector<char> backing_data;
            uint32_t full_size = sizeof(MultiLayerMainHeader) + sizeof(MultiLayerSideHeader) + d.getEncodedSize();
            backing_data.resize(full_size);
            
            struct MultiLayerMainHeader mh { 1, min_pt.x, min_pt.y, min_pt.z, max_pt.x, max_pt.y, max_pt.z };
            struct MultiLayerSideHeader sh { d.getLayerId(), d.getEncodedSize() };

            uint32_t offset = 0;
            memcpy(backing_data.data(), &mh, sizeof(mh));
            offset += sizeof(mh);
            memcpy(backing_data.data() + offset, &sh, sizeof(sh));
            offset += sizeof(sh);
            memcpy(backing_data.data() + offset, d.get_buffer().data(), d.getEncodedSize());
            offset += d.getEncodedSize();
            this->transporter->SendEncodedData(full_size, backing_data.data(), true, client_id);

            std::unique_lock<std::mutex> lock(m);
            (*completed_jobs)++;
            if (*completed_jobs == n_clients) {
                cv.notify_all();
            }
            ResultWriter::addSendTimestamp(curr_frame_nr, client_id, start_time.count());
            
            });
    }
    // Wait until all jobs complete
    cv.wait(lk, [this, completed_jobs, n_clients] {return *completed_jobs == n_clients; });
   
    ResultWriter::setFrameReadyToSave(curr_frame_nr);
	return 0;
}

