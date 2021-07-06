import numpy as np



def getUniqueSolution(poses,Q_p3p,Point4,impoint4):

    try:
        # Keep only rotation and translations for which
        # the camera projections are all within the camera
        R = []
        T = []
        for i in range (0, int(np.size(poses,1)/4)):
            tR = poses[:,4*i+1:4*i+4].T
            tT = -tR @ poses[:,4*i]
            proj = tR @ Q_p3p + np.tile(tT.T, (3,1)).T
            if np.min(proj[2,:]) > 0:
                R.append(tR)
                T.append(tT)

        # Disambiguate using a 4th entry of Q and the
        # corresponding q
        extrinsicIdx = -1
        err_best = 99999
        for i in range (0,len(T)):
            proj = R[i] @ Point4 + T[i]
            proj = proj / proj[-1]
            proj = proj[0:-1]
            err = np.linalg.norm(proj - impoint4, 2)
            if err < err_best:
                extrinsicIdx = i
                err_best = err

        if extrinsicIdx == -1:
            valid = False
            R_estim = 0
            t_estim = 0
        else:
            R_estim = R[extrinsicIdx]
            t_estim = T[extrinsicIdx]
            valid = True
    except IndexError:
        valid = False
        R_estim = 0
        t_estim = 0

    return R_estim, t_estim, valid

# P3P implmentation based on the paper
# "A Novel Parametrization of the Perspective-Three-Point
# Problem for a Direct Computation of Absolute Camera
# Position and Orientation" - Kneip et. al.

def p3p(worldPoints, imageVectors):
    # Empty matrix of max. 4 rotations and translations
    poses = np.zeros((3,16))
    # Individual world points
    P1 = worldPoints[:,0]
    P2 = worldPoints[:,1]
    P3 = worldPoints[:,2]
    # Vectors between world points
    vect1 = P2 - P1
    vect2 = P3 - P1
    # Check non-collinearity of these vectors
    if np.linalg.norm(np.cross(vect1,vect2),2) == 0:
        print("Error: The 3 points used in P3P must be non-collinear")
        return

    # Idividual feature vectors in the nu frame
    f1 = imageVectors[:,0]
    f2 = imageVectors[:,1]
    f3 = imageVectors[:,2]
    # Compute an orthogonal basis for the tau frame
    # and then invert it. The resulting matrix T converts
    # from frame nu to frame tau
    tx = f1
    tz = np.cross(f1,f2) / np.linalg.norm(np.cross(f1,f2),2)
    ty = np.cross(tz,tx)
    T = np.c_[tx, ty, tz].T
    # Transform f3 from nu to tau
    f3_tau = T @ f3
    # We desire f3_tau[2] (the z-comp.) to be < 0. So, if it
    # is bigger than 0, we inverse f1 and f2 and P1 and P2
    if f3_tau[2] > 0:
        # We essentially exchange f1 and f2
        f1 = imageVectors[:,1]
        f2 = imageVectors[:,0]
        f3 = imageVectors[:,2]
        # We essentially exchange P1 and P2
        P1 = worldPoints[:,1]
        P2 = worldPoints[:,0]
        P3 = worldPoints[:,2]
        # Recompute inverse of basis of tau as before
        tx = f1
        tz = np.cross(f1,f2) / np.linalg.norm(np.cross(f1,f2),2)
        ty = np.cross(tz,tx)
        T = np.c_[tx, ty, tz].T
        # Transform f3 from nu to tau
        f3_tau = T @ f3

    # Compute an orthogonal basis for the eta frame
    # and then invert it. The resulting matrix N converts
    # from the world frame to frame nu
    nx = (P2 - P1) / np.linalg.norm(P2 - P1,2)
    nz = np.cross(nx,(P3 - P1)) / np.linalg.norm(np.cross(nx,(P3 - P1)),2)
    ny = np.cross(nz,nx)
    N = np.c_[nx, ny, nz].T
    # Testing checkpoint

    # Convert P3 from the world frame to the nu frame
    P3_nu = N @ (P3 - P1)
    p_1 = P3_nu[0];
    p_2 = P3_nu[1];
    # Lenght of vector P2-P1
    d_12 = np.linalg.norm(P2 - P1,2)
    # Define phis
    phi_1 = f3_tau[0] / f3_tau[2]
    phi_2 = f3_tau[1] / f3_tau[2]
    # Define b = cot(beta)
    cos_beta = f1.T @ f2
    b = 1/(1 - cos_beta**2) - 1
    if cos_beta < 0:
        b = -np.sqrt(b)
    else:
        b = np.sqrt(b)
    # Some auxiliary variables that are helpful to type less
    phi_1_pw2 = phi_1**2;
    phi_2_pw2 = phi_2**2;
    p_1_pw2 = p_1**2;
    p_1_pw3 = p_1_pw2 * p_1;
    p_1_pw4 = p_1_pw3 * p_1;
    p_2_pw2 = p_2**2;
    p_2_pw3 = p_2_pw2 * p_2;
    p_2_pw4 = p_2_pw3 * p_2;
    d_12_pw2 = d_12**2;
    b_pw2 = b**2;
    # Computation of factors of 4th degree polynomial
    factor_4 = -phi_2_pw2*p_2_pw4 \
               -p_2_pw4*phi_1_pw2 \
               -p_2_pw4

    factor_3 = 2*p_2_pw3*d_12*b \
               +2*phi_2_pw2*p_2_pw3*d_12*b \
               -2*phi_2*p_2_pw3*phi_1*d_12

    factor_2 = -phi_2_pw2*p_2_pw2*p_1_pw2 \
               -phi_2_pw2*p_2_pw2*d_12_pw2*b_pw2 \
               -phi_2_pw2*p_2_pw2*d_12_pw2 \
               +phi_2_pw2*p_2_pw4 \
               +p_2_pw4*phi_1_pw2 \
               +2*p_1*p_2_pw2*d_12 \
               +2*phi_1*phi_2*p_1*p_2_pw2*d_12*b \
               -p_2_pw2*p_1_pw2*phi_1_pw2 \
               +2*p_1*p_2_pw2*phi_2_pw2*d_12 \
               -p_2_pw2*d_12_pw2*b_pw2 \
               -2*p_1_pw2*p_2_pw2

    factor_1 = 2*p_1_pw2*p_2*d_12*b \
               +2*phi_2*p_2_pw3*phi_1*d_12 \
               -2*phi_2_pw2*p_2_pw3*d_12*b \
               -2*p_1*p_2*d_12_pw2*b

    factor_0 = -2*phi_2*p_2_pw2*phi_1*p_1*d_12*b \
               +phi_2_pw2*p_2_pw2*d_12_pw2 \
               +2*p_1_pw3*d_12 \
               -p_1_pw2*d_12_pw2 \
               +phi_2_pw2*p_2_pw2*p_1_pw2 \
               -p_1_pw4 \
               -2*phi_2_pw2*p_2_pw2*p_1*d_12 \
               +p_2_pw2*phi_1_pw2*p_1_pw2 \
               +phi_2_pw2*p_2_pw2*d_12_pw2*b_pw2

    # Compute roots of fourth order polynomial using
    # Ferrari's closed-form solution. The roots may be
    # complex and in that case only the real parts should
    # be kept
    factors = np.array([factor_4, factor_3, factor_2,
                        factor_1, factor_0])
    x = roots4thOrder(factors)
    x = x.real
    # Testing checkpoint

    # Backsubtitute solutions in other equations
    for i in range(0,4):

        cot_alpha = (-phi_1*p_1/phi_2 - x[i]*p_2 + d_12*b) \
                    /(-phi_1*x[i]*p_2 / phi_2 + p_1 - d_12)

        cos_theta = x[i]
            
        sin_theta = np.sqrt(1 - x[i]**2)
        sin_alpha = np.sqrt(1 / (cot_alpha**2 + 1))
        cos_alpha = np.sqrt(1 - sin_alpha**2)

        if cot_alpha < 0:
            cos_alpha = -cos_alpha
        # Build C_nu
        C_nu = np.array([d_12*cos_alpha*(sin_alpha*b + cos_alpha),
                         cos_theta*d_12*sin_alpha*(sin_alpha*b + cos_alpha),
                         sin_theta*d_12*sin_alpha*(sin_alpha*b + cos_alpha)])
        # Compute C
        C = P1 + N.T @ C_nu
        # Bulid Q
        Q = np.array([[-cos_alpha, -sin_alpha*cos_theta, -sin_alpha*sin_theta],
                      [sin_alpha, -cos_alpha*cos_theta, -cos_alpha*sin_theta],
                      [0, -sin_theta, cos_theta]])
        # Compute R
        R = N.T @ Q.T @ T

        poses[:,4*i] = C
        poses[:,4*i+1:4*i+4] = R
    return poses

def roots4thOrder(factors):
    A = factors[0]
    B = factors[1]
    C = factors[2]
    D = factors[3]
    E = factors[4]

    A_pw2 = A*A
    B_pw2 = B*B
    A_pw3 = A_pw2*A
    B_pw3 = B_pw2*B
    A_pw4 = A_pw3*A
    B_pw4 = B_pw3*B

    alpha = -3*B_pw2/(8*A_pw2) + C/A
    beta = B_pw3/(8*A_pw3) - B*C/(2*A_pw2) + D/A
    gamma = -3*B_pw4/(256*A_pw4) + B_pw2*C/(16*A_pw3) \
            -B*D/(4*A_pw2) + E/A

    alpha_pw2 = alpha*alpha
    alpha_pw3 = alpha_pw2*alpha

    # Use np.cbrt(x) and not x**(1/3) to make
    # sure the real root is rerurned
    P = -alpha_pw2/12 - gamma
    Q = -alpha_pw3/108 + alpha*gamma/3 - beta**2/8 
    R = -Q/2 + np.sqrt(Q**2 / 4 + P**3 / 27)
    U = np.cbrt(R)

    if U == 0:
        y = -5*alpha/6 - np.cbrt(Q)
    else:
        y = -5*alpha/6 - P/(3*U) + U

    w = np.sqrt(alpha + 2*y)

    # np.lib.scimath.sqrt computes compex roots for negative
    # numbers
    roots = np.zeros(4, dtype='complex')
    roots[0] = -B/(4*A) + \
        0.5*(w + np.lib.scimath.sqrt(-(3*alpha + 2*y + 2*beta/w)))
    roots[1] = -B/(4*A) + \
        0.5*(w - np.lib.scimath.sqrt(-(3*alpha + 2*y + 2*beta/w)))
    roots[2] = -B/(4*A) + \
        0.5*(-w + np.lib.scimath.sqrt(-(3*alpha + 2*y - 2*beta/w)))
    roots[3] = -B/(4*A) + \
        0.5*(-w - np.lib.scimath.sqrt(-(3*alpha + 2*y - 2*beta/w)))
    return roots