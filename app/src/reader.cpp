/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: MIT
 */

#include "reader.hpp"

#include <iostream>
#include <reader_interiornet.hpp>
#include <reader_kitti.hpp>
#include <reader_leica.hpp>
#include <reader_newercollege.hpp>
#include <reader_openni.hpp>
#include <reader_raw.hpp>
#include <reader_tum.hpp>
#include <se/common/filesystem.hpp>

se::Reader* se::create_reader(const se::Reader::Config& config)
{
    se::Reader* reader = nullptr;
    switch (config.reader_type) {
    case se::ReaderType::OPENNI:
        reader = new se::OpenNIReader(config);
        break;
    case se::ReaderType::RAW:
        reader = new se::RAWReader(config);
        break;
    case se::ReaderType::NEWERCOLLEGE:
        reader = new se::NewerCollegeReader(config);
        break;
    case se::ReaderType::TUM:
        reader = new se::TUMReader(config);
        break;
    case se::ReaderType::INTERIORNET:
        reader = new se::InteriorNetReader(config);
        break;
    case se::ReaderType::LEICA:
        reader = new se::LeicaReader(config);
        break;
    case se::ReaderType::KITTI:
        reader = new se::KITTIReader(config);
        break;
    default:
        std::cerr << "Error: Unrecognised file format, file not loaded\n";
    }
    // Handle failed initialization
    if (reader && !reader->good()) {
        delete reader;
        reader = nullptr;
    }
    return reader;
}
