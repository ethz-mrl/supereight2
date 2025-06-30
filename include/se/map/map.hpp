/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2023 Nils Funk
 * SPDX-FileCopyrightText: 2019-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_HPP
#define SE_MAP_HPP

#include <optional>
#include <se/common/eigen_utils.hpp>
#include <se/common/yaml.hpp>
#include <se/map/io/octree_io.hpp>
#include <se/map/octree/octree.hpp>
#include <se/map/raycaster.hpp>



namespace se {

// Forward Declaration
template<typename DataT = se::Data<Field::TSDF, Colour::Off, Id::Off>,
         Res ResT = Res::Single,
         int BlockSize = 8>
class Map;



template<Field FldT, Colour ColB, Id IdB, Res ResT, int BlockSize>
class Map<se::Data<FldT, ColB, IdB>, ResT, BlockSize> {
    public:
    typedef Octree<Data<FldT, ColB, IdB>, ResT, BlockSize> OctreeType;
    typedef typename OctreeType::DataType DataType;
    typedef typename OctreeType::DataConfigType DataConfigType;
    typedef typename OctreeType::BlockType BlockType;
    typedef typename OctreeType::NodeType NodeType;
    typedef typename OctreeType::SurfaceMesh SurfaceMesh;
    typedef typename OctreeType::StructureMesh StructureMesh;

    struct Config {
        /** The dimensions of the map in metres.
         */
        Eigen::Vector3f dim = Eigen::Vector3f::Constant(10.0f);

        /** The resolution of map voxels in metres.
         */
        float res = 0.1f;

        /** The transformation from the world frame W to the map frame M.
         */
        Eigen::Isometry3f T_MW = Eigen::Isometry3f(Eigen::Translation3f(dim / 2));

        /** Reads the struct members from the "map" node of a YAML file. Members not present in the YAML
         * file aren't modified.
         */
        void readYaml(const std::string& yaml_file);

        // The definition of this function MUST be inside the definition of Config for template
        // argument deduction to work.
        friend std::ostream& operator<<(std::ostream& os, const Config& c)
        {
            os << str_utils::volume_to_pretty_str(c.dim, "dim") << " m\n";
            os << str_utils::value_to_pretty_str(c.res, "res") << " m/voxel\n";
            os << str_utils::eigen_matrix_to_pretty_str(c.T_MW.matrix(), "T_MW") << "\n";
            return os;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /**
     * \brief The map constructor.
     *
     * \note Default map origin is set at the centre of the map.
     *
     * \param[in] dim             The dimension of the map in [meter]
     * \param[in] res             The resolution of the map in [meter/voxel]
     * \param[in] data_config     The configuration file for the data
     */
    Map(const Eigen::Vector3f& dim,
        const float res,
        const typename Data<FldT, ColB, IdB>::Config& data_config =
            typename Data<FldT, ColB, IdB>::Config());

    /**
     * \brief The map constructor.
     *
     * \param map_config          The configuration file for the map (e.g. map dimension, resolution and origin)
     * \param data_config         The configuration file for the data
     */
    Map(const Config& map_config,
        const typename Data<FldT, ColB, IdB>::Config& data_config =
            typename Data<FldT, ColB, IdB>::Config());

    /** The copy constructor is explicitly deleted.
     */
    Map(const Map&) = delete;

    /** The copy assignment operator is explicitly deleted.
     */
    Map& operator=(const Map&) = delete;

    /**
     * \brief Verify if a point is inside the map.
     *
     * \param[in] point_W         The point to be verified
     * \return True if the point is inside the map, false otherwise
     */
    bool contains(const Eigen::Vector3f& point_W) const;

    /**
     * \brief Get the transformation from world to map frame
     *
     * \return T_MW
     */
    const Eigen::Isometry3f& getTMW() const
    {
        return T_MW_;
    };

    /**
     * \brief Get the transformation from map to world frame
     *
     * \return T_WM
     */
    const Eigen::Isometry3f& getTWM() const
    {
        return T_WM_;
    };

    /**
     * \brief Get the dimensions of the map in [meter] (length x width x height)
     *
     * \return The dimensions of the map
     */
    const Eigen::Vector3f& getDim() const
    {
        return dimension_;
    }

    /**
     * \brief Get the resolution of the map in [meter/voxel]
     *
     * \return The resolution of the map
     */
    float getRes() const
    {
        return resolution_;
    }

    /**
     * \brief Get the data configuration of the map.
     *
     * \return The data configuration of the map
     */
    const DataConfigType& getDataConfig() const
    {
        return data_config_;
    }

    /**
     * \brief Get the stored data at the provided coordinates in [meter].
     *
     * \tparam SafeB          The parameter turning "contains point" verification on and off (Off by default)
     * \param[in]  point_W    The coordinates of the point in world frame [meter] to evaluate
     * \return The data at the provided coordinates
     */
    template<Safe SafeB = Safe::Off>
    DataType getData(const Eigen::Vector3f& point_W) const;

    /**
     * \brief Get the stored min data at the provided coordinates in [meter] for a given scale.
     *
     * \tparam SafeB          The parameter turning "contains point" verification on and off (Off by default)
     * \tparam ResTDummy      The dummy parameter disabling the function off for single res and TSDF maps // TODO: Clean up with C++20 using required
     * \param point_W         The coordinates of the point in world frame [meter] to accessed
     * \param scale_desired   The scale to be accessed
     * \return The min data at the provided coordinates and scale
     */
    template<Safe SafeB = Safe::Off, Res ResTDummy = ResT>
    typename std::enable_if_t<ResTDummy == Res::Multi, DataType>
    getMinData(const Eigen::Vector3f& point_W, const int scale_desired) const;

    /**
     * \brief Get the stored max data at the provided coordinates in [meter] for a given scale.
     *
     * \tparam SafeB          The parameter turning "contains point" verification on and off (Off by default)
     * \tparam ResTDummy      The dummy parameter disabling the function off for single res and TSDF maps // TODO: Clean up with C++20 using required
     * \param point_W         The coordinates of the point in world frame [meter] to accessed
     * \param scale_desired   The scale to be accessed
     * \return The max data at the provided coordinates and scale
     */
    template<Safe SafeB = Safe::Off, Res ResTDummy = ResT>
    typename std::enable_if_t<ResTDummy == Res::Multi, DataType>
    getMaxData(const Eigen::Vector3f& point_W, const int scale_desired) const
    {
        Eigen::Vector3i voxel_coord;

        if constexpr (SafeB == Safe::Off) // Evaluate at compile time
        {
            pointToVoxel<Safe::Off>(point_W, voxel_coord);
        }
        else {
            if (!pointToVoxel<Safe::On>(point_W, voxel_coord)) {
                return DataType();
            }
        }

        return se::visitor::getMaxData(octree_, voxel_coord, scale_desired);
    }

    /** Interpolate a member of se::Data at \p point_W and scale \p desired_scale.
     *
     * \tparam SafeB              \p point_W will be bounds-checked if set to se::Safe::On.
     * \param[in] point_W         The coordinates, expressed in the world frame W in metres, of the
     *                            point the data will be interpolated at.
     * \param[in] valid           A functor with the following declaration, returning whether the
     *                            supplied data is valid and should be used for interpolation:
     *                            \code{.cpp}
     *                            template<se::Field FldT, se::Colour ColB, se::Id IdB>
     *                            bool valid(const typename se::Data<FldT, ColB, IdB>& data);
     *                            \endcode
     * \param[in] get             A functor with the following declaration, returning the data of
     *                            type \p T to be interpolated:
     *                            \code{.cpp}
     *                            template<se::Field FldT, se::Colour ColB, se::Id IdB>
     *                            T get(const typename se::Data<FldT, ColB, IdB>& data);
     *                            \endcode
     *                            Type \p T must implement the following operators:
     *                            \code{.cpp}
     *                            T operator+(const T& a, const T& b);
     *                            T operator*(const T& a, const float b);
     *                            \endcode
     * \param[in]  desired_scale  The finest scale the data should be interpolated at. Ignored for
     *                            se::Res:Single maps.
     * \param[out] returned_scale The actual scale the data was interpolated at will be stored into
     *                            \p *returned_scale if \p returned_scale is non-null. \p
     *                            *returned_scale is not modified if \p std::nullopt is returned.
     *                            The value of \p *returned_scale will not be less than \p
     *                            desired_scale in se::Res::Multi maps.
     * \return The interpolated data if valid, \p std::nullopt otherwise.
     */
    template<typename ValidF, typename GetF, Safe SafeB = Safe::Off>
    std::optional<std::invoke_result_t<GetF, DataType>>
    interp(const Eigen::Vector3f& point_W,
           ValidF valid,
           GetF get,
           const Scale desired_scale = 0,
           Scale* const returned_scale = nullptr) const;

    /** Interpolate the field at \p point_W and \p desired_scale. See #interp() for more detailed
     * documentation.
     */
    template<Safe SafeB = Safe::Off>
    std::optional<field_t> interpField(const Eigen::Vector3f& point_W,
                                       const Scale desired_scale = 0,
                                       Scale* const returned_scale = nullptr) const;

    /** Interpolate the colour at \p point_W and \p desired_scale. See #interp() for more detailed
     * documentation.
     */
    template<Safe SafeB = Safe::Off, Colour ColourTDummy = ColB>
    typename std::enable_if_t<ColourTDummy == Colour::On, std::optional<colour_t>>
    interpColour(const Eigen::Vector3f& point_W,
                 const Scale desired_scale = 0,
                 Scale* const returned_scale = nullptr) const;



    /** Return the gradient of a member of se::Data at \p point_W and scale \p desired_scale.
     *
     * \tparam SafeB              \p point_W will be bounds-checked if set to se::Safe::On.
     * \param[in] point_W         The coordinates, expressed in the world frame W in metres, of the
     *                            point the data gradient will be computed at.
     * \param[in] valid           A functor with the following declaration, returning whether the
     *                            supplied data is valid and should be used for gradient computation:
     *                            \code{.cpp}
     *                            template<se::Field FldT, se::Colour ColB, se::Id IdB>
     *                            bool valid(const typename se::Data<FldT, ColB, IdB>& data);
     *                            \endcode
     * \param[in] get             A functor with the following declaration, returning the data of
     *                            type \p T the gradient of which will be computed:
     *                            \code{.cpp}
     *                            template<se::Field FldT, se::Colour ColB, se::Id IdB>
     *                            T get(const typename se::Data<FldT, ColB, IdB>& data);
     *                            \endcode
     *                            Type \p T must implement the following operators:
     *                            \code{.cpp}
     *                            T operator+(const T& a, const T& b);
     *                            T operator-(const T& a, const T& b);
     *                            T operator*(const T& a, const float b);
     *                            \endcode
     * \param[in]  desired_scale  The finest scale the data gradient should be computed at. Ignored
     *                            for se::Res::Single maps.
     * \param[out] returned_scale The actual scale the gradient was computed at will be stored into
     *                            \p *returned_scale if \p returned_scale is non-null. \p
     *                            *returned_scale is not modified if \p std::nullopt is returned.
     *                            The value of \p *returned_scale will not be less than \p
     *                            desired_scale in se::Res::Multi maps.
     * \return The data gradient if valid, \p std::nullopt otherwise.
     */
    template<typename ValidF, typename GetF, Safe SafeB = Safe::Off>
    std::optional<Eigen::Matrix<std::invoke_result_t<GetF, DataType>, 3, 1>>
    grad(const Eigen::Vector3f& point_W,
         ValidF valid,
         GetF get,
         const Scale desired_scale = 0,
         Scale* const returned_scale = nullptr) const;

    /** Return the field gradient at \p point_W and \p desired_scale. See #grad() for more
     * detailed documentation.
     */
    template<Safe SafeB = Safe::Off>
    std::optional<field_vec_t> getFieldGrad(const Eigen::Vector3f& point_W,
                                            const Scale desired_scale = 0,
                                            Scale* const returned_scale = nullptr) const;

    /**
     * \brief Save three slices of the field value, each perpendicular to one of the axes (x, y and
     * z) at the provided coordinates. Setting any of the filenames to the empty string will skip
     * saving the respective slice.
     *
     * \note Only VTK (`.vtk`) files are currently supported.
     *
     * \param[in] filename_x The file where the slice perpendicular to the x axis will be written.
     * \param[in] filename_y The file where the slice perpendicular to the y axis will be written.
     * \param[in] filename_z The file where the slice perpendicular to the z axis will be written.
     * \param[in] point_W    The point in the world frame in units of meters where the slices
     *                       intersect. The x coordinate denotes the position along the x axis that
     *                       the slice perpendicular to the x axis will be computed etc.
     * \return Zero on success and non-zero on error.
     */
    int saveFieldSlices(const std::string& filename_x,
                        const std::string& filename_y,
                        const std::string& filename_z,
                        const Eigen::Vector3f& point_W) const;

    /**
     * \brief Save three slices of the minimum field value, each perpendicular to one of the axes
     * (x, y and z) at the provided coordinates. Setting any of the filenames to the empty string
     * will skip saving the respective slice.
     *
     * \note Only VTK (`.vtk`) files are currently supported.
     *
     * \param[in] filename_x The file where the slice perpendicular to the x axis will be written.
     * \param[in] filename_y The file where the slice perpendicular to the y axis will be written.
     * \param[in] filename_z The file where the slice perpendicular to the z axis will be written.
     * \param[in] point_W    The point in the world frame in units of meters where the slices
     *                       intersect. The x coordinate denotes the position along the x axis that
     *                       the slice perpendicular to the x axis will be computed etc.
     * \param[in] scale      The minimum scale the minimum field values will be extracted from.
     * \return Zero on success and non-zero on error.
     */
    template<se::Field FldTDummy = FldT>
    typename std::enable_if_t<FldTDummy == se::Field::Occupancy, int>
    saveMinFieldSlices(const std::string& filename_x,
                       const std::string& filename_y,
                       const std::string& filename_z,
                       const Eigen::Vector3f& point_W,
                       const int scale) const;

    /**
     * \brief Save three slices of the maximum field value, each perpendicular to one of the axes
     * (x, y and z) at the provided coordinates. Setting any of the filenames to the empty string
     * will skip saving the respective slice.
     *
     * \note Only VTK (`.vtk`) files are currently supported.
     *
     * \param[in] filename_x The file where the slice perpendicular to the x axis will be written.
     * \param[in] filename_y The file where the slice perpendicular to the y axis will be written.
     * \param[in] filename_z The file where the slice perpendicular to the z axis will be written.
     * \param[in] point_W    The point in the world frame in units of meters where the slices
     *                       intersect. The x coordinate denotes the position along the x axis that
     *                       the slice perpendicular to the x axis will be computed etc.
     * \param[in] scale      The minimum scale the maximum field values will be extracted from.
     * \return Zero on success and non-zero on error.
     */
    template<se::Field FldTDummy = FldT>
    typename std::enable_if_t<FldTDummy == se::Field::Occupancy, int>
    saveMaxFieldSlices(const std::string& filename_x,
                       const std::string& filename_y,
                       const std::string& filename_z,
                       const Eigen::Vector3f& point_W,
                       const int scale) const;

    /**
     * \brief Save three slices of the integration scale, each perpendicular to one of the axes (x,
     * y and z) at the provided coordinates. Setting any of the filenames to the empty string will
     * skip saving the respective slice.
     *
     * \note Only VTK (`.vtk`) files are currently supported.
     *
     * \param[in] filename_x The file where the slice perpendicular to the x axis will be written.
     * \param[in] filename_y The file where the slice perpendicular to the y axis will be written.
     * \param[in] filename_z The file where the slice perpendicular to the z axis will be written.
     * \param[in] point_W    The point in the world frame in units of meters where the slices
     *                       intersect. The x coordinate denotes the position along the x axis that
     *                       the slice perpendicular to the x axis will be computed etc.
     * \return Zero on success and non-zero on error.
     */
    template<Res ResTDummy = ResT>
    typename std::enable_if_t<ResTDummy == Res::Multi, int>
    saveScaleSlices(const std::string& filename_x,
                    const std::string& filename_y,
                    const std::string& filename_z,
                    const Eigen::Vector3f& point_W) const;

    /** Return a mesh of the reconstructed surface in the world frame in units of metres. Apply a
     * transformation, from the world frame W to some output frame O, \p T_OW to each mesh vertex.
     * For se::Res::Multi maps, only data at scale \p min_desired_scale or coarser will be used to
     * generate the mesh. This allows generating a coarser mesh which is less demanding in terms of
     * computational time and memory. The value of \p min_desired_scale has no effect on
     * se::Res::Single maps.
     */
    SurfaceMesh mesh(const Eigen::Affine3f& T_OW = Eigen::Affine3f::Identity(),
                     const int min_desired_scale = 0) const;

    /** Save the mesh returned by se::Map::mesh() in \p filename. The \p T_OW and \p
     * min_desired_scale arguments are passed directly to se::Map::mesh(). The file format will be
     * selected based on the extension of \p filename, which must be one of those in
     * se::io::mesh_extensions. Return the value returned by se::io::save_mesh().
     */
    int saveMesh(const std::string& filename,
                 const Eigen::Affine3f& T_OW = Eigen::Affine3f::Identity(),
                 const int min_desired_scale = 0) const;

    /** Return a mesh of the octree structure in the world frame in units of metres. Apply a
     * transformation, from the world frame W to some output frame O, \p T_OW to each mesh vertex.
     * The \p only_leaves argument is passed directly to se::octree_structure_mesh().
     */
    StructureMesh structure(const Eigen::Affine3f& T_OW = Eigen::Affine3f::Identity(),
                            const bool only_leaves = true) const;

    /** Save the mesh returned by se::Map::structure_meshing() in \p filename. The \p T_OW and \p
     * only_leaves arguments are passed directly to se::Map::structure(). The file format will be
     * selected based on the extension of \p filename, which must be one of those in
     * se::io::mesh_extensions. Return the value returned by se::io::save_mesh().
     */
    int saveStructure(const std::string& filename,
                      const Eigen::Affine3f& T_OW = Eigen::Affine3f::Identity(),
                      const bool only_leaves = true) const;

    /**
     * \brief Convert voxel coordinates in [voxel] to its centre point coordinates in [meter].
     *
     * \warning The function assumes the voxel has size 1 (i.e. scale 0).
     *
     * \param[in]  voxel_coord    The voxel coordinates in [voxel] to be converted
     * \param[out] point_W        The converted centre point coordinates in [meter]
     */
    void voxelToPoint(const Eigen::Vector3i& voxel_coord, Eigen::Vector3f& point_W) const;

    /**
     * \brief Convert voxel coordinates in [voxel] for a given voxel size to its centre point coordinates in [meter].
     *
     * \param[in]  voxel_coord    The voxel coordinates in [voxel] to be converted
     * \param[in]  voxel_size     The size of the voxel in [voxel]
     * \param[out] point_W        The converted centre point coordinates in [meter]
     */
    void voxelToPoint(const Eigen::Vector3i& voxel_coord,
                      const int voxel_size,
                      Eigen::Vector3f& point_W) const;

    /**
     * \brief Convert voxel coordinates in [voxel] for a given voxel size to its eight corner point coordinates in [meter].
     *
     * \warning The function assumes the voxel has size 1 (i.e. scale 0).
     *
     * \param[in]  voxel_coord        The voxel coordinates in [voxel] to be converted
     * \param[out] corner_points_W    The converted centre point coordinates in [meter]
     */
    void voxelToCornerPoints(const Eigen::Vector3i& voxel_coord,
                             Eigen::Matrix<float, 3, 8>& corner_points_W) const;

    /**
     * \brief Convert voxel coordinates in [voxel] for a given voxel size to its eight corner point coordinates in [meter].
     *
     * \param[in]  voxel_coord        The voxel coordinates in [voxel] to be converted
     * \param[in]  voxel_size         The size of the voxel in [voxel]
     * \param[out] corner_points_W    The converted centre point coordinates in [meter]
     */
    void voxelToCornerPoints(const Eigen::Vector3i& voxel_coord,
                             const int voxel_size,
                             Eigen::Matrix<float, 3, 8>& corner_points_W) const;

    /**
     * \brief Convert point coordinates in [meter] to its voxel coordinates (bottom, front, left corner) in [voxel]
     *
     * \note Use as `map. template pointToVoxel<se::Safe::Off>(...)
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  point_W        The point coordinates in [meter] to be converted
     * \param[out] voxel_coord    The converted voxel coordinates (bottom, front, left corner) in [voxel]
     * \return True if the point inside the map, false otherwise
     */
    template<se::Safe SafeB = se::Safe::On>
    typename std::enable_if_t<SafeB == se::Safe::On, bool>
    pointToVoxel(const Eigen::Vector3f& point_W, Eigen::Vector3i& voxel_coord) const;

    /**
     * \brief Convert point coordinates in [meter] to its voxel coordinates (bottom, front, left corner) in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  point_W        The point coordinates in [meter] to be converted
     * \param[out] voxel_coord    The converted voxel coordinates (bottom, front, left corner) in [voxel]
     * \return True
     */
    template<se::Safe SafeB>
    typename std::enable_if_t<SafeB == se::Safe::Off, bool>
    pointToVoxel(const Eigen::Vector3f& point_W, Eigen::Vector3i& voxel_coord) const;

    /**
     * \brief Convert point coordinates in [meter] to its voxel coordinates in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  point_W        The point coordinates in [meter] to be converted
     * \param[out] voxel_coord_f  The converted voxel coordinates in [voxel]
     * \return True if the point inside the map, false otherwise
     */
    template<se::Safe SafeB = se::Safe::On>
    typename std::enable_if_t<SafeB == se::Safe::On, bool>
    pointToVoxel(const Eigen::Vector3f& point_W, Eigen::Vector3f& voxel_coord_f) const;

    /**
     * \brief Convert point coordinates in [meter] to its voxel coordinates in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  point_W        The point coordinates in [meter] to be converted
     * \param[out] voxel_coord_f  The converted voxel coordinates in [voxel]
     * \return True
     */
    template<se::Safe SafeB>
    typename std::enable_if_t<SafeB == se::Safe::Off, bool>
    pointToVoxel(const Eigen::Vector3f& point_W, Eigen::Vector3f& voxel_coord_f) const;

    /**
     * \brief Convert a vector of point coordinates in [meter] to its voxel coordinates in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  points_W       The vector of point coordinates in [meter] to be converted
     * \param[out] voxel_coords   The vector of converted voxel coordinates in [voxel]
     * \return True if all points are inside the map, false otherwise
     */
    template<se::Safe SafeB = se::Safe::On>
    typename std::enable_if_t<SafeB == se::Safe::On, bool> pointsToVoxels(
        const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points_W,
        std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& voxel_coords)
        const;

    /**
     * \brief Convert a vector of point coordinates in [meter] to its voxel coordinates in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  points_W       The vector of point coordinates in [meter] to be converted
     * \param[out] voxel_coords   The vector of converted voxel coordinates in [voxel]
     * \return True
     */
    template<se::Safe SafeB>
    typename std::enable_if_t<SafeB == se::Safe::Off, bool> pointsToVoxels(
        const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points_W,
        std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& voxel_coords)
        const;

    /** Return a reference to the internal se::Octree. */
    OctreeType& getOctree()
    {
        return octree_;
    };

    /** Return a constant reference to the internal se::Octree. */
    const OctreeType& getOctree() const
    {
        return octree_;
    };

    /** Return the axis-aligned bounding box in the world frame W of the map's allocated leaves. It
     * can be used to test if some point expressed in the world frame W is contained in it using
     * Eigen::AlignedBox3f::contains().
     */
    const Eigen::AlignedBox3f& aabb() const;

    static constexpr Field fld_ = FldT;
    static constexpr Colour col_ = ColB;
    static constexpr Id id_ = IdB;

    static constexpr Res res_ = ResT;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    OctreeType octree_;
    const float resolution_;          ///< The resolution of the map
    const Eigen::Vector3f dimension_; ///< The dimensions of the map
    const Eigen::Isometry3f T_MW_;    ///< The transformation from world to map frame
    const Eigen::Isometry3f T_WM_;    ///< The transformation from map to world frame

    const Eigen::Vector3f lb_M_; ///< The lower map bound
    const Eigen::Vector3f ub_M_; ///< The upper map bound

    const DataConfigType data_config_; ///< The configuration of the data

    mutable Eigen::AlignedBox3f cached_aabb_;
    mutable Eigen::AlignedBox3i cached_octree_aabb_;

    /** The eight relative unit corner offsets */
    static const Eigen::Matrix<float, 3, 8> corner_rel_steps_;
};

//// Full alias template for alternative setup
template<se::Field FldT = se::Field::TSDF,
         se::Colour ColB = se::Colour::Off,
         se::Id IdB = se::Id::Off,
         se::Res ResT = se::Res::Single,
         int BlockSize = 8>
using MapD = Map<Data<FldT, ColB, IdB>, ResT, BlockSize>;


// Occupancy map setups
template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyMap = Map<OccupancyData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyColMap = Map<OccupancyColData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyIdMap = Map<OccupancyIdData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyColIdMap = Map<OccupancyColIdData, ResT, BlockSize>;


// TSDF map setups
template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFMap = Map<TSDFData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFColMap = Map<TSDFColData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFIdMap = Map<TSDFIdData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFColIdMap = Map<TSDFColIdData, ResT, BlockSize>;

} // namespace se

#include "impl/map_impl.hpp"

#endif // SE_MAP_HPP
