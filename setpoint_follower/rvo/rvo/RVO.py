import numpy as np

class RVO :
    tau = 100
    radius = .06
    margin = 0
    nb_sample = 24
    test_velocities = []
    magnitude = (1, 2/3, 1/3)
    rvo_detect = np.inf

    def __init__(self, dim : int, speed, dist_com) :
        # RVO.rvo_detect = dist_com
        if dim == 2 :
            angles = ((np.cos(2*i*np.pi/RVO.nb_sample), np.sin(2*i*np.pi/RVO.nb_sample), 0) for i in range(RVO.nb_sample))
            RVO.test_velocities.append(np.array((0, 0, 0)))
            for a in angles :
                for m in RVO.magnitude :
                    RVO.test_velocities.append(speed * m * np.array(a))
        elif dim == 3 :
            golden_ratio = (1+np.sqrt(5))/2
            for i in range(RVO.nb_sample) :
                theta = 2*np.pi*i*golden_ratio
                phi = np.arccos(1-2*i/RVO.nb_sample)
                v = np.array((np.cos(theta)*np.sin(phi), np.sin(phi)*np.sin(theta), np.cos(phi)))
                RVO.test_velocities.append(v)
                RVO.test_velocities.append(.5*v)

    @classmethod
    def is_in_vo(cls, pos, idx, other_idx, v_test) :
        v_norm = np.linalg.norm(v_test)
        if v_norm == 0 :
            return 0
        v = v_test/v_norm
        dp = pos[other_idx] - pos[idx]
        lambda_ = dp @ v
        if lambda_ < 0 :
            lambda_ = 0
        elif lambda_ > cls.tau * v_norm :
            lambda_ = cls.tau*v_norm
        if (np.linalg.norm(dp-lambda_*v) <= 2*cls.radius + cls.margin) :
            if lambda_ == 0 :
                if dp@v > 0 :
                    return 0
                return np.inf
            return v_norm/lambda_
        return 0

    @classmethod
    def update_rvo(cls, pos, vel, idx, v_opt) :
        v_tests = [v for v in cls.test_velocities] + [v_opt[idx]]
        v_tests.sort(key = lambda x : np.linalg.norm(v_opt[idx] - x))
        costs = [0 for _ in v_tests]
        for other_idx, pos_other in enumerate(pos) :
            if (other_idx != idx and np.linalg.norm(pos[idx] - pos_other) <= cls.rvo_detect) :
                for i, v in enumerate(v_tests) :
                    res = cls.is_in_vo(pos, idx, other_idx, 2*v-vel[idx]-vel[other_idx])
                    costs[i] += res # type: ignore

        return v_tests[np.argmin(costs)]
