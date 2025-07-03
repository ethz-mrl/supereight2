/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_HPP
#define SE_DATA_HPP

#include <array>
#include <se/map/data_colour.hpp>
#include <se/map/data_field.hpp>
#include <se/map/data_id.hpp>

namespace se {

template<Field FldT = Field::TSDF, Colour ColB = Colour::Off, Id IdB = Id::Off>
struct Data {
    typedef FieldData<FldT> FieldType;
    typedef ColourData<ColB> ColourType;
    typedef IdData<IdB> IdType;

    FieldType field;
    [[no_unique_address]] ColourType colour;
    [[no_unique_address]] IdType id;

    struct Config {
        typename FieldType::Config field;
        typename ColourType::Config colour;
        typename IdType::Config id;

        /** Reads the struct members from the "data" node of a YAML file. Members not present in the
         * YAML file aren't modified.
         */
        void readYaml(const std::string& yaml_file)
        {
            field.readYaml(yaml_file);
            colour.readYaml(yaml_file);
            id.readYaml(yaml_file);
        }

        // The definition of this function MUST be inside the definition of Config for template
        // argument deduction to work.
        friend std::ostream& operator<<(std::ostream& os, const Config& c)
        {
            os << c.field;
            os << c.colour;
            os << c.id;
            return os;
        }
    };

    static constexpr Field fld_ = FldT;
    static constexpr Colour col_ = ColB;
    static constexpr Id id_ = IdB;
    static constexpr bool normals_along_gradient = FieldData<FldT>::normals_along_gradient;
    static constexpr field_t surface_boundary = FieldData<FldT>::surface_boundary;
};



namespace data {

/** Up-propagate the mean of the valid \p child_data into \p parent_data and return the number of
 * children with valid data.
 */
template<Field FldT, Colour ColB, Id IdB>
int up_prop_mean(Data<FldT, ColB, IdB>& parent_data,
                 const std::array<Data<FldT, ColB, IdB>, 8>& child_data);

/** Up-propagate the minimum of the valid \p child_data into \p parent_data and return the number of
 * children with valid data.
 */
template<Field FldT, Colour ColB, Id IdB>
int up_prop_min(Data<FldT, ColB, IdB>& parent_min_data,
                const std::array<Data<FldT, ColB, IdB>, 8>& child_min_data);

/** Up-propagate the maximum of the valid \p child_data into \p parent_data and return the number of
 * children with valid data.
 */
template<Field FldT, Colour ColB, Id IdB>
int up_prop_max(Data<FldT, ColB, IdB>& parent_max_data,
                const std::array<Data<FldT, ColB, IdB>, 8>& child_max_data);

} // namespace data

template<Field FldT, Colour ColB, Id IdB>
inline void set_invalid(Data<FldT, ColB, IdB>& data);

template<Colour ColB, Id IdB>
inline void set_invalid(Data<Field::TSDF, ColB, IdB>& data)
{
    data = Data<Field::TSDF, ColB, IdB>();
}

template<Colour ColB, Id IdB>
inline void set_invalid(Data<Field::Occupancy, ColB, IdB>& data)
{
    data = Data<Field::Occupancy, ColB, IdB>();
}



template<Field FldT, Colour ColB, Id IdB>
inline bool is_valid(const Data<FldT, ColB, IdB>& data)
{
    return data.field.valid();
}



template<Field FldT, Colour ColB, Id IdB>
inline field_t get_field(const Data<FldT, ColB, IdB>& data);

template<Colour ColB, Id IdB>
inline field_t get_field(const Data<Field::TSDF, ColB, IdB>& data)
{
    return data.field.tsdf;
}

template<Colour ColB, Id IdB>
inline field_t get_field(const Data<Field::Occupancy, ColB, IdB>& data)
{
    return data.field.occupancy * data.field.weight;
}



template<Field FldT, Colour ColB, Id IdB>
inline bool is_inside(const Data<FldT, ColB, IdB>& data);

template<Colour ColB, Id IdB>
inline bool is_inside(const Data<Field::TSDF, ColB, IdB>& data)
{
    return get_field(data) < Data<Field::TSDF, ColB, IdB>::surface_boundary;
}

template<Colour ColB, Id IdB>
inline bool is_inside(const Data<Field::Occupancy, ColB, IdB>& data)
{
    return get_field(data) > Data<Field::Occupancy, ColB, IdB>::surface_boundary;
}



// Occupancy data setups
typedef Data<Field::Occupancy, Colour::Off, Id::Off> OccupancyData;
typedef Data<Field::Occupancy, Colour::On, Id::Off> OccupancyColData;
typedef Data<Field::Occupancy, Colour::Off, Id::On> OccupancyIdData;
typedef Data<Field::Occupancy, Colour::On, Id::On> OccupancyColIdData;

// TSDF data setups
typedef Data<Field::TSDF, Colour::Off, Id::Off> TSDFData;
typedef Data<Field::TSDF, Colour::On, Id::Off> TSDFColData;
typedef Data<Field::TSDF, Colour::Off, Id::On> TSDFIdData;
typedef Data<Field::TSDF, Colour::On, Id::On> TSDFColIdData;

} // namespace se

#include "impl/data_impl.hpp"

#endif // SE_DATA_HPP
