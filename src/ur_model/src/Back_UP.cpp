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

        cc::HomogeneousTransformation T_0_B() const;

    /** 
    * @brief fixed base wrt to robot base
    */
    cc::HomogeneousTransformation T_B_0() const;

    /**
     * @brief Tool wrt ef
     */
    cc::HomogeneousTransformation T_Tool_Ef() const;

    /** 
    * @brief gravity vector wrt word
    */
    cc::LinearPosition g_B() const;

    /** 
    * @brief gravity vector wrt base
    */
    cc::LinearPosition g_0() const;

    /** 
    * @brief endeffector transformation matrix
    */
    cc::HomogeneousTransformation T_ef_0(const cc::JointPosition &q) const;

    /** 
    * @brief endeffector transformation matrix wrt to robot base frame
    */
    cc::HomogeneousTransformation T_ef_B(const cc::JointPosition &q) const;

    /** 
    * @brief tool transformation matrix
    */
    cc::HomogeneousTransformation T_tool_0(const cc::JointPosition &q) const;

    /** 
    * @brief endeffector transformation matrix wrt to robot base frame
    */
    cc::HomogeneousTransformation T_tool_B(const cc::JointPosition &q) const;

    /** 
    * @brief endeffector jacobian matrix wrt to robot base frame
    */
    cc::Jacobian J_ef_0(const cc::JointPosition &q) const;

    /** 
    * @brief endeffector jacobian matrix wrt to robot base frame
    */
    cc::Jacobian J_tool_0(const cc::JointPosition &q) const;

    /** 
    * @brief jacobian matrix at j-th joint offseted by length tj wrt 0 frame 
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian Jt_j_0(const cc::LinearPosition &tj, const cc::JointPosition &q, int j) const;

    /** 
    * @brief endeffector jacobian derivative wrt robot base frame 
    */
    cc::Jacobian Jp_ef_0(const cc::JointPosition &q, const cc::JointVelocity &qP) const;

    /** 
    * @brief tool jacobian derivative wrt robot base frame 
    */
    cc::Jacobian Jp_tool_0(const cc::JointPosition &q, const cc::JointVelocity &qP) const;

    /** 
    * @brief jacobian derivative at j-th joint wrt robot base frame 
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian Jp_j_0(const cc::JointPosition &q, const cc::JointVelocity &qP, int j) const;

    /** 
    * @brief jacobian derivative at j-th joint offseted by length tj wrt robot base frame 
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian Jtp_j_0(const cc::LinearPosition &tj, const cc::JointPosition &q, const cc::JointVelocity &qP, int j) const;

    /** 
    * @brief jacobian derivative at j-th center of mass wrt robot base frame 
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian Jpcm_j_0(const cc::JointPosition &q, const cc::JointVelocity &qP, int j) const;


/** 
    * @brief jacobian derivative at j-th center of mass offseted by length tj wrt robot base frame 
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian Jtpcm_j_0(const cc::LinearPosition &tj, const cc::JointPosition &q, const cc::JointVelocity &qP, int j) const;

    /**
     * @brief return lower joint limits of j-th joint
     * 
     */
    cc::Scalar lowerJointLimits_j(int j) const;

    /**
     * @brief return upper joint limits of j-th joint
     * 
     */
    cc::Scalar upperJointLimits_j(int j) const;

    /** 
    * @brief inverse kinematics, return all posible 8 ik solutions
    * 
    * @note Simon: I think there is a bug here! Need to check ...
    */
    void inverseKinematics(IKSolutions &Qs, const cc::HomogeneousTransformation &Tef, cc::Scalar q6_d) const;

    /**
     * @brief Get base Frame name
     */
    std::string getBaseFrame() const;

    /**
     * @brief Get base Frame name
     */
    std::string get0Frame() const;

    /**
     * @brief Get the Tool Frame name
     */
    std::string getToolFrame() const;

    /**
     * @brief Get the DH Frame name
     */
    std::string getDHFrame_j(int j) const;

    /**
     * @brief broad cast frames in tf tree
     */
    void broadcastFrames(const cc::JointPosition &q, const ros::Time &time);


    /* inverse kinematics */
    void matrix_IK(IKSolutions &Qs, const cc::HomogeneousTransformation &Tef, cc::Scalar q6_d) const;

// model.cpp
bool URModel::init(ros::NodeHandle &nh)











