# [C++] ur_model

## 注意
- 每个函数（类似matrix_M）分为2部分
    - public: const + return
    - private: 用于更新数值

## 省略的部分
    // joint limits
    cc::Scalar lo_jl1, lo_jl2, lo_jl3, lo_jl4, lo_jl5, lo_jl6;
    cc::Scalar hi_jl1, hi_jl2, hi_jl3, hi_jl4, hi_jl5, hi_jl6;

    // gravity vectors
    cc::LinearPosition g_b_, g_0_;

    // base to world transformation
    cc::HomogeneousTransformation T_0_b_;
    cc::HomogeneousTransformation T_b_0_;

    // tool to ef transformation
    cc::HomogeneousTransformation T_tool_ef_;

    // frames
    std::string robot_0_frame_; // 0 frame tf name
    std::string base_frame_;    // base frame tf name
    std::string tool_frame_;    // tool frame tf name

    // broadcast frames
    tf::TransformBroadcaster br_;
    std::vector<tf::StampedTransform> tf_stamped_transform_; // stack of tf transformations


## 我的函数（名称）
        typedef model_interface::ModelBase Base;
        typedef cc::MatrixX Regressor_theta;
        typedef cc::VectorX Regressor_Yr;

        Regressor_theta Theta_;
        Regressor_Yr Yr_;