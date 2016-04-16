using PCLCommon
using PCLIO
using PCLFilters
using Base.Test

table_scene_lms400_path = joinpath(dirname(@__FILE__), "data",
    "table_scene_lms400.pcd")

@testset "pcl::filters" begin
    cloud = PointCloud{PointXYZ}(table_scene_lms400_path)
    cloud_filtered = PointCloud{PointXYZ}()

    uniform_sampling = UniformSampling{PointXYZ}()
    setInputCloud(uniform_sampling, cloud)
    setRadiusSearch(uniform_sampling, 0.01)
    filter(uniform_sampling, cloud_filtered)
    @test length(cloud_filtered) < length(cloud)

    for sor in [
        VoxelGrid{PointXYZ}(),
        ApproximateVoxelGrid{PointXYZ}()
        ]
        setInputCloud(sor, cloud)
        setLeafSize(sor, 0.01, 0.01, 0.01)
        filter(sor, cloud_filtered)
        @test length(cloud_filtered) < length(cloud)
    end

    sor = StatisticalOutlierRemoval{PointXYZ}()
    setInputCloud(sor, cloud)
    setMeanK(sor, 10)
    setStddevMulThresh(sor, 1.0)
    filter(sor, cloud_filtered)
    @test length(cloud_filtered) < length(cloud)

    r = RadiusOutlierRemoval{PointXYZ}()
    setInputCloud(r, cloud)
    setRadiusSearch(r, 0.2)
    setMinNeighborsInRadius(r, 5)
    filter(r, cloud_filtered)
    @test length(cloud_filtered) < length(cloud)
end
