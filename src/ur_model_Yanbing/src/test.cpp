void URModel::caculet_Jcm6_0_dot(cc::Jacobian &Jcm6_0_dot,
                        const cc::JointPosition &q
                        const cc::JointVelocity &qp) const
{
    // Initial
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            Jcm6_0_dot(i, j) = 0;
        }
    }
}