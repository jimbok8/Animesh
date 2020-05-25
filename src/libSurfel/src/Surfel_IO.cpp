
#include "Surfel_IO.h"
#include "Surfel.h"

#include <GeomFileUtils/io_utils.h>
#include <iostream>
#include <string>
#include <vector>

/*
	********************************************************************************
	**																			  **
	**					Load and Save    										  **
	**																			  **
	********************************************************************************
*/


/**
 * Save surfel data as binary file to disk
 */
void
save_surfels_to_file(const std::string &file_name,
                     const std::vector<std::shared_ptr<Surfel>> &surfels) {
    using namespace std;

    cout << "Saving..." << flush;

    ofstream file{file_name, ios::out | ios::binary};
    // Count
    write_unsigned_int(file, surfels.size());
    for (auto const &surfel : surfels) {
        // ID
        write_string(file, surfel->id);
        // FrameData size
        write_unsigned_int(file, surfel->frame_data.size());
        for (auto const &fd : surfel->frame_data) {
            // PixelInFrame
            write_size_t(file, fd.pixel_in_frame.pixel.x);
            write_size_t(file, fd.pixel_in_frame.pixel.y);
            write_size_t(file, fd.pixel_in_frame.frame);
            write_float(file, fd.depth);

            // Transform
            write_float(file, fd.transform(0, 0));
            write_float(file, fd.transform(0, 1));
            write_float(file, fd.transform(0, 2));
            write_float(file, fd.transform(1, 0));
            write_float(file, fd.transform(1, 1));
            write_float(file, fd.transform(1, 2));
            write_float(file, fd.transform(2, 0));
            write_float(file, fd.transform(2, 1));
            write_float(file, fd.transform(2, 2));

            // Normal
            write_vector_3f(file, fd.normal);
        }
        write_unsigned_int(file, surfel->neighbouring_surfels.size());
        for (const auto &surfel_ptr : surfel->neighbouring_surfels) {
            write_string(file, surfel_ptr->id);
        }
        write_vector_3f(file, surfel->tangent);
    }
    file.close();
    cout << " done." << endl;
}

/**
 * Load surfel data from binary file
 */
void
load_from_file(const std::string &file_name, std::vector<Surfel> &surfels) {
    using namespace std;

    cout << "Loading from file " << file_name << "..." << flush;

    ifstream file{file_name, ios::in | ios::binary};
    surfels.clear();

    unsigned int num_surfels = read_unsigned_int(file);
    cout << "  loading " << num_surfels << " surfels" << endl;
    unsigned int pct5 = num_surfels / 20;

    map<string, vector<string>> neighbours_of_surfel_by_id;
    map<string, shared_ptr<Surfel>> surfel_ptr_by_id;

    for (int sIdx = 0; sIdx < num_surfels; ++sIdx) {
        if (sIdx % pct5 == 0) {
            cout << "." << flush;
        }
        string surfel_id = read_string(file);


        unsigned int num_frames = read_unsigned_int(file);
        vector<FrameData> frames;
        for (int fdIdx = 0; fdIdx < num_frames; ++fdIdx) {
            FrameData fd;

            // PixelInFrame
            fd.pixel_in_frame.pixel.x = read_size_t(file);
            fd.pixel_in_frame.pixel.y = read_size_t(file);

            fd.pixel_in_frame.frame = read_size_t(file);
            fd.depth = read_float(file);

            // Transform
            float m[9];
            for (float &mIdx : m) {
                mIdx = read_float(file);
            }
            fd.transform << m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8];

            // Normal
            fd.normal = read_vector_3f(file);
            frames.push_back(fd);
        }

        unsigned int num_neighbours = read_unsigned_int(file);
        vector<string> neighbours;
        neighbours.reserve(num_neighbours);
        for (int nIdx = 0; nIdx < num_neighbours; ++nIdx) {
            neighbours.push_back(read_string(file));
        }

        const auto tangent = read_vector_3f(file);

        Surfel surfel{surfel_id, frames, vector<shared_ptr<Surfel>>{}, tangent};
        neighbours_of_surfel_by_id.emplace(surfel_id, neighbours);
        surfel_ptr_by_id.emplace(surfel_id, make_shared<Surfel>(surfel));
        surfels.push_back(surfel);

    }
    file.close();

    // Populate neighbours
    for( auto& surfel : surfels) {
        const auto& neighbours = neighbours_of_surfel_by_id.at(surfel.id);
        for( const auto& nid : neighbours ) {
            surfel.neighbouring_surfels.push_back(surfel_ptr_by_id.at(nid));
        }
    }

    cout << endl << " done." << endl;
}