import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class FDMassiveMIMOChannel:
    """
    3D Full-Dimension MIMO Channel

    Scenario: 3D-UMa (Urban Macro Cell) and frequencies below 6GHz

    Coordinate System
    Elevation (theta)
    +z: theta = 0 deg (points to zenith)
    -z: theta = 180 deg (therefore, theta=90 points to horizon)

    +x: phi = 0

    Other assumptions:
    BS located at (x=0, y=0, z=25) meters
    """

    def __init__(self,
                 bs_ant_row=8,
                 bs_ant_col=8,
                 bs_height=25,
                 freq=3.6e9,
                 ue_num=1,
                 bs_num=1,
                 bw=5e6,
                 print_info=False):

        # General
        self.ue_num = ue_num
        self.bs_num = bs_num
        self.bs_ant = bs_ant_row * bs_ant_col   # Number of antennas per base station
        self.ue_ant = 1                         # Number of antennas per user (Assume single-antenna users)
        self.n_bs_ant_row = bs_ant_row
        self.n_bs_ant_col = bs_ant_col
        self.bs_height = bs_height
        self.freq = freq
        self.wavelength = 3e8/freq
        self.bw = bw                            # System bandwidth
        self.cell_radius = 100                  # Meters
        self.cell_span_angle = 180              # Degrees
        self.min_bs_ue_dist = 35                # Meters
        self.pol_angles = [-45, 45]             # -45 deg, +45 deg
        self.print_info = print_info

        # Antenna pattern
        self.ant_pattern_azimuth = []
        self.ant_pattern_elevation = []
        self.ant_pattern_dB = []
        self.num_angles = 0
        self.naz = 0                            # Number of azimuth angles
        self.nel = 0                            # Number of alevation angles

        # Load patterns
        print("1/5 Loading antenna patterns")
        self.load_ant_patterns()

        # Radiation (Field) Patterns
        # Local coord
        self.F_theta_lcs = np.empty((len(self.pol_angles), self.num_angles))
        self.F_phi_lcs = np.empty((len(self.pol_angles), self.num_angles))
        # Global coord
        self.F_bs_theta_gcs = np.empty((bs_num, len(self.pol_angles), self.naz, self.nel))
        self.F_bs_phi_gcs = np.empty((bs_num, len(self.pol_angles), self.naz, self.nel))
        self.F_ue_theta_gcs = np.empty((bs_num, ue_num, len(self.pol_angles), self.naz, self.nel))
        self.F_ue_phi_gcs = np.empty((bs_num, ue_num, len(self.pol_angles), self.naz, self.nel))

        # Get radiation fields
        print("2/5 Computing radiation fields from antenna patterns")
        for polIdx, pol in enumerate(self.pol_angles):
            F_theta, F_phi = self.get_radiation_fields(self.ant_pattern_dB[polIdx], pol)
            self.F_theta_lcs[polIdx, :] = F_theta
            self.F_phi_lcs[polIdx, :] = F_phi

        # Mobility
        self.ue_speed = np.empty((bs_num, ue_num))              # km/hr
        self.ue_direction = np.empty((bs_num, ue_num, 2))       # direction of motion [phi, theta]

        # LOS angles
        self.zod_los_deg = np.empty((bs_num, ue_num))           # Elevation
        self.zoa_los_deg = np.empty((bs_num, ue_num))           # Elevation
        self.aod_los_deg = np.empty((bs_num, ue_num))           # Azimuth
        self.aoa_los_deg = np.empty((bs_num, ue_num))           # Azimuth

        # Coordinates
        self.bs_coord = np.empty((bs_num, 3))
        self.ue_coord = np.empty((bs_num, ue_num, 3))
        self.ue_isIndoor = np.empty((bs_num, ue_num))           # Flag indicating whether UE is indoors or outdoors
        self.ue_isLOS = np.empty((bs_num, ue_num))              # Flag indicating whether UE is LOS or NLOS
        self.ue_chan_params = np.empty((bs_num, ue_num), dtype=object)
        self.distance_3d = np.empty((bs_num, ue_num))
        self.distance_2d = np.empty((bs_num, ue_num))

        # BS and UE orientation with respect to global coordinate [alpha=Bearing angle, beta=Downtilt, gamma=Slant]
        self.bs_orientation_deg = np.empty((bs_num, 3))
        self.ue_orientation_deg = np.empty((bs_num, ue_num, 3))

        # Antenna spacing
        self.bs_ant_space_row = 0.0625                               # Larger distance in the vertical dimension        # FIXME - check actual distance, using Xing's value right now
        self.bs_ant_space_col = (3e8/3.8e9)/2                        # RENEW array antenna separation: lambda/2 @ 3.8GHz
        self.array_geometry = np.empty(bs_num, dtype=object)         # Vector. Each entry is a 2D matrix (horiz, vert)

        # Geometry
        print("3/5 Generate scenario geometry")
        self.scenario_setup()

    def scenario_setup(self):
        """

        == NOTES ==
        1) BS located at XY origin
        2) Primed variables used to indicate LCS
        3) TODO: Only tested with one BS. Need to extend to more (and fix coordinates if BS no longer at origin)

            z
            |
            |
            |----- y
          /
        /
       x

        """
        num_ue = self.ue_num
        num_bs = self.bs_num
        bs_height = self.bs_height

        # Array geometry
        ant_row_vec = np.cumsum(self.bs_ant_space_row * np.ones((1, self.n_bs_ant_row))) - self.bs_ant_space_row
        ant_col_vec = np.cumsum(self.bs_ant_space_col * np.ones((1, self.n_bs_ant_col))) + bs_height - self.bs_ant_space_col
        ant_array = np.empty((len(ant_row_vec)*len(ant_col_vec), 3))            # 3D
        cnt = 0
        for i, ival in enumerate(ant_row_vec):
            for j, jval in enumerate(ant_col_vec):
                ant_array[cnt, :] = [ival, 0, jval]
                cnt += 1

        # BSs
        for idxbs in range(num_bs):

            # Currently only one BS with array pointing towards phi=0 (+x)
            # BS coordinates [x, y, z]
            self.bs_coord[idxbs, :] = [0, 0, bs_height]
            # BS Orientations (Page 15) [alpha=Bearing angle, beta=Downtilt angle, gamma=Slant angle]
            self.bs_orientation_deg[idxbs, :] = [90, 0, 0]
            # Give BS antenna field patterns Frx and Ftx in the global coordinate system and array geometries
            # [BS ID][Angles, # Polarizations]
            F_theta, F_phi = self.translate_field_coords(self.bs_orientation_deg[idxbs, :])
            F_theta_t = np.transpose(F_theta)
            F_phi_t = np.transpose(F_phi)
            for polIdx in range(len(self.pol_angles)):
                self.F_bs_theta_gcs[idxbs, polIdx, :, :] = np.reshape(F_theta_t[polIdx, :], (self.naz, self.nel))
                self.F_bs_phi_gcs[idxbs, polIdx, :, :] = np.reshape(F_phi_t[polIdx, :], (self.naz, self.nel))
            self.array_geometry[idxbs] = np.array(ant_array)                                                            #FIXME - Fix array orientation. Currently is y=0 all the time and starts at x=0 and z=0

            # UEs
            for idxue in range(num_ue):
                loc, isIndoor = self.place_ue()
                self.ue_coord[idxbs, idxue] = loc
                self.ue_isIndoor[idxbs, idxue] = isIndoor

                # Give 3D locations of BS and UT, and determine LOS AOD (ϕLOS,AOD), LOS ZOD (θLOS,ZOD), LOS AOA
                # (ϕLOS,AOA), and LOS ZOA (θLOS,ZOA) of each BS and UT in the global coordinate system
                # TODO: add tile angle to bs antenna array
                self.distance_3d[idxbs, idxue] = np.sqrt(loc[0] ** 2 + loc[1] ** 2 + (bs_height-loc[2]) ** 2)
                self.distance_2d[idxbs, idxue] = np.sqrt(loc[0] ** 2 + loc[1] ** 2)

                h_ue = loc[2]
                d_2D = self.distance_2d[idxbs, idxue]

                # Determine if LOS or NLOS
                isLOS = False
                if not isIndoor:
                    d_2D_out = d_2D
                    pr_los = self.los_probability(h_ue, d_2D_out)
                    if np.random.uniform(0, 1) >= (1 - pr_los):
                        isLOS = True

                self.ue_isLOS[idxbs, idxue] = isLOS
                self.ue_chan_params[idxbs, idxue] = self.chan_model_params(isLOS, d_2D, h_ue)

                # Assuming angles for downlink (Departure from BS, arrival at UE). Need to reverse for uplink
                # Elevation
                self.zod_los_deg[idxbs, idxue] = 180 - (np.arccos((bs_height-loc[2]) / self.distance_3d[idxbs, idxue]) / np.pi * 180)
                self.zoa_los_deg[idxbs, idxue] = np.arccos((bs_height-loc[2]) / self.distance_3d[idxbs, idxue]) / np.pi * 180

                # Azimuth
                if loc[0] < 0:
                    # Second quadrant
                    self.aod_los_deg[idxbs, idxue] = 180 - (np.arccos(abs(loc[0]) / self.distance_2d[idxbs, idxue]) / np.pi * 180)
                    self.aoa_los_deg[idxbs, idxue] = 360 - (np.arccos(abs(loc[0]) / self.distance_2d[idxbs, idxue]) / np.pi * 180)
                else:
                    # First quadrant
                    self.aod_los_deg[idxbs, idxue] = np.arccos(loc[0] / self.distance_2d[idxbs, idxue]) / np.pi * 180
                    self.aoa_los_deg[idxbs, idxue] = 180 + (np.arccos(loc[0] / self.distance_2d[idxbs, idxue]) / np.pi * 180)

                # Give BS and UT array orientations with respect to the global coordinate system.
                # BS and UT array orientation defined by [alpha=bearing, beta=downtilt, gamma=slant]
                # Azimuth pointing to BS (XY orig)
                self.ue_orientation_deg[idxbs, idxue, :] = [self.aoa_los_deg[idxbs, idxue], 0, 0]

                # Give UT antenna field patterns Frx and Ftx in the global coordinate system and array geometries
                F_theta, F_phi = self.translate_field_coords(self.ue_orientation_deg[idxbs, idxue, :])
                F_theta_t = np.transpose(F_theta)
                F_phi_t = np.transpose(F_phi)
                for polIdx in range(len(self.pol_angles)):
                    self.F_ue_theta_gcs[idxbs, idxue, polIdx, :, :] = np.reshape(F_theta_t[polIdx, :], (self.naz, self.nel))
                    self.F_ue_phi_gcs[idxbs, idxue, polIdx, :, :] = np.reshape(F_phi_t[polIdx, :], (self.naz, self.nel))

                # Give speed and direction of motion of UT in the global coordinate system
                self.ue_speed[idxbs, idxue] = 0
                self.ue_direction[idxbs, idxue, :] = [0, 0]   # [Phi, Theta]

    def get_rotation_matrix(self, alpha, beta, gamma):
        # Eq. (7.1-5)
        R00 = np.cos(alpha) * np.cos(beta)
        R01 = np.cos(alpha) * np.sin(beta) * np.sin(gamma) - np.sin(alpha) * np.cos(gamma)
        R02 = np.cos(alpha) * np.sin(beta) * np.cos(gamma) + np.sin(alpha) * np.sin(gamma)
        R10 = np.sin(alpha) * np.cos(beta)
        R11 = np.sin(alpha) * np.sin(beta) * np.sin(gamma) + np.cos(alpha) * np.cos(gamma)
        R12 = np.sin(alpha) * np.sin(beta) * np.cos(gamma) - np.cos(alpha) * np.sin(gamma)
        R20 = -np.sin(beta)
        R21 = np.cos(beta) * np.sin(gamma)
        R22 = np.cos(beta) * np.cos(gamma)

        R = [[R00, R01, R02],
             [R10, R11, R12],
             [R20, R21, R22]]

        return R

    def translate_polar_coords(self, xyz, orientation_deg, direction):
        """
        Pages 16, 17
        :param xyz:
        :param alpha:
        :param beta:
        :param gamma:
        :param direction:
        :return:
        """
        alpha = np.radians(orientation_deg[0])
        beta = np.radians(orientation_deg[1])
        gamma = np.radians(orientation_deg[2])

        R = self.get_rotation_matrix(alpha, beta, gamma)

        if direction == "reverse":
            # Given theta and phi of a point in GCS, find corresponding position in LCS
            R = np.linalg.inv(R)

        theta = np.arccos(np.dot([0, 0, 1], np.dot(R, xyz))) / np.pi * 180
        phi = np.angle(np.dot([1, 1j, 0], np.dot(R, xyz))) / np.pi * 180

        return theta, phi

    def translate_field_coords(self, orientation_deg):
        """

        :param orientation_deg:
        :param pol_vec:          Polarization vector, one entry per polarization
        :param gamma:

        :return:
        """

        # self.ant_pattern_elevation   # Main lobe z=90
        # self.ant_pattern_azimuth     # Main lobe x=0
        alpha = np.radians(orientation_deg[0])
        beta = np.radians(orientation_deg[1])
        gamma = np.radians(orientation_deg[2])

        pol_vec = self.pol_angles

        theta_all = np.unique(self.ant_pattern_elevation)
        phi_all = np.unique(self.ant_pattern_azimuth)
        F_theta_gcs = np.zeros((len(theta_all) * len(phi_all), len(pol_vec)))
        F_phi_gcs = np.zeros((len(theta_all) * len(phi_all), len(pol_vec)))

        for polIdx in range(len(pol_vec)):
            count = 0
            for phi_idx, phi in enumerate(phi_all):
                for theta_idx, theta in enumerate(theta_all):
                    phi = np.radians(phi)
                    theta = np.radians(theta)

                    # Eq. (7.1-15) in page 18
                    a = np.sin(gamma) * np.cos(theta) * np.sin(phi-alpha) + np.cos(gamma) * (np.cos(beta) * np.sin(theta) - np.sin(beta) * np.cos(theta) * np.cos(phi-alpha))
                    b = np.sin(gamma) * np.cos(phi-alpha) + np.sin(beta) * np.cos(gamma) * np.sin(phi-alpha)
                    psi = np.angle(a + 1j * b)
                    cos_psi = np.cos(psi)
                    sin_psi = np.sin(psi)

                    F_theta_gcs[count, polIdx], F_phi_gcs[count, polIdx] = np.dot(np.array([[cos_psi, -sin_psi], [sin_psi, cos_psi]]),
                                                          np.array([self.F_theta_lcs[polIdx][count], self.F_phi_lcs[polIdx][count]]))           # FIXME - revisit this
                    count += 1

        """       
        F_theta_lcs_array = np.reshape(self.F_theta_lcs, (360, 181))
        F_phi_lcs_array = np.reshape(self.F_theta_lcs, (360, 181))
        F_theta_gcs_array = np.reshape(F_theta_gcs, (360, 181))
        F_phi_gcs_array = np.reshape(F_theta_gcs, (360, 181))

        fig = plt.figure(111)
        ax = fig.add_subplot(111)
        im = plt.imshow(np.transpose(ant_patt), cmap='jet')
        plt.colorbar(im)
        ax.set_xlabel('Azimuth (phi)')
        ax.set_ylabel('Elevation (theta)')
        plt.show(block=False)

        fig = plt.figure(112)
        ax1 = fig.add_subplot(121)
        im1 = plt.imshow(np.transpose(F_theta_lcs_array), cmap='jet')
        plt.colorbar(im1)
        ax1.set_xlabel('Azimuth (phi)')
        ax1.set_ylabel('Elevation (theta)')
        ax1.set_title('LOCAL')
        plt.show(block=False)
        ax2 = fig.add_subplot(122)
        im2 = plt.imshow(np.transpose(F_theta_gcs_array), cmap='jet')
        plt.colorbar(im2)
        ax2.set_xlabel('Azimuth (phi)')
        ax2.set_ylabel('Elevation (theta)')
        ax2.set_title('GLOBAL')
        plt.show(block=True)
        """
        return F_theta_gcs, F_phi_gcs

    def place_ue(self):
        """
            z
            |
            |
            |----- y
          /
        /
       x

        :return: ue_pos - x,y,z coordinate of UE
                 indoor - flag indicating whether UE is indoors or outdoors

        UEs will be in the +y (XY quadrants 1 and 2, i.e., -x and +x) region (if a max cell_span_angle of 180)
        NOTE: based on Table 6-1, page 14 in 3D-UMa TR36.873
        Min UE-eNB 2D distance of 35m (Also in Page 20 of 3GPP TR 38.901 version 14.3.0 Release 14)
        """

        if self.min_bs_ue_dist < 35:
            self.min_bs_ue_dist = 35

        ue_pos = np.zeros(3, dtype=np.float64)  # [x, y, z]
        # get ue height
        # Number of floors in outdoor
        fl_num = 1
        isIndoor = self.is_indoor()
        if isIndoor:
            fl_num = np.random.uniform(1, np.random.uniform(4, 8))

        ue_height = 3 * (fl_num - 1) + 1.5

        # UE Position
        ue_pos[2] = ue_height  # bs_height - ue_height

        # get UE ground position - uniform in spanned area
        while ue_pos[1] == 0 or \
                ue_pos[0] ** 2 + ue_pos[1] ** 2 > self.cell_radius ** 2 or \
                ue_pos[0] ** 2 + ue_pos[1] ** 2 < self.min_bs_ue_dist ** 2 or \
                abs(ue_pos[0]) / ue_pos[1] > np.tan(self.cell_span_angle / 2 / 180 * np.pi):
            ue_pos[0] = np.random.uniform(-1 * self.cell_radius,
                                          self.cell_radius)
            ue_pos[1] = np.random.uniform(0, self.cell_radius)

        return ue_pos, isIndoor

    def load_ant_patterns(self):
        """
        Read antenna gain file (txt)

        Output:
        Main lobe pointing to +x.
        ant_pattern_azimuth 0 deg at +x [0, 360]
        ant_pattern_elevation 0 deg at +z [0, 180]
        :return:
        """
        filename_pol1 = 'Ant_3625MHz_neg45.txt'
        filename_pol2 = 'Ant_3625MHz_pos45.txt'
        path = './antennaPatterns/3625/'

        # [Power (dBm), Azimuth Angle [-180,180], Elevation Angle [0, 180]]]
        # Size of matrix is (2N,3) where N is 361*181=65341. Ignore second half of data
        N = len(range(-180, 180+1)) * len(range(0, 180+1))
        data_pol1 = np.loadtxt(path+filename_pol1, dtype='float', delimiter=',')
        data_pol2 = np.loadtxt(path+filename_pol2, dtype='float', delimiter=',')
        data_pol1 = data_pol1[0:(N-181), :]  # +- 180 repetition
        data_pol2 = data_pol2[0:(N-181), :]  # +- 180 repetition

        # Normalize
        pwr_lin = 10**(data_pol1[:, 0]/10)
        # pwr_norm = pwr_lin / np.max(pwr_lin)
        pwr_m45 = pwr_lin  # 10 * np.log10(pwr_norm)

        pwr_lin = 10**(data_pol2[:, 0]/10)
        pwr_norm = pwr_lin / np.max(pwr_lin)
        pwr_p45 = pwr_lin  # 10 * np.log10(pwr_norm)
        pwr_tmp_db = data_pol2[:, 0]

        # Translate into Global Coordinate System
        ndata = 180 * 181
        ant_pattern_azimuth_tmp = np.mod(data_pol1[:, 1] + 360, 360)        # Convert from [-180, 180] to [0, 360]
        self.ant_pattern_azimuth = np.concatenate((ant_pattern_azimuth_tmp[ndata::], ant_pattern_azimuth_tmp[0:ndata]))
        self.ant_pattern_elevation = np.flipud(data_pol1[:, 2])  # +z=180deg in dataset. Change to 0deg
        # self.ant_pattern_azimuth = np.mod(self.ant_pattern_azimuth + 90, 360) # Rotate +90deg, Facing +y instead of +x
        ant_pattern_pwr_dB_p45 = np.concatenate((pwr_p45[ndata::], pwr_p45[0:ndata]))
        ant_pattern_pwr_dB_m45 = np.concatenate((pwr_m45[ndata::], pwr_m45[0:ndata]))

        self.ant_pattern_dB = np.array([ant_pattern_pwr_dB_m45, ant_pattern_pwr_dB_p45])
        self.num_angles = 360 * 181
        self.naz = len(np.unique(self.ant_pattern_azimuth))
        self.nel = len(np.unique(self.ant_pattern_elevation))

        # plt.figure()
        # plt.plot(self.ant_pattern_elevation, 'b')
        # plt.plot(self.ant_pattern_azimuth, '--r')
        # plt.plot(ant_pattern_azimuth_tmp, 'k')
        # plt.show()

    def get_radiation_fields(self, pattern, pol_ang_deg):
        """
        Polarized antennas
        Section 7.3.2 Page 23, Model-2, Eqs. (7.3-4) and (7.3-5)
        :return:
        """
        F_theta_lcs = np.sqrt(pattern) * np.cos(np.radians(pol_ang_deg))
        F_phi_lcs = np.sqrt(pattern) * np.sin(np.radians(pol_ang_deg))
        return F_theta_lcs, F_phi_lcs

    @staticmethod
    def is_indoor():
        """
        Indoor UE fraction is 80%
        :return: indoor yes/no
        """
        if np.random.uniform(0, 1) > 0.2:
            return True
        else:
            return False

    def los_probability(self, h_ut, d_2D_out):
        """
        Line-Of-Sight Probability
        :param h_ut: User terminal (UT) heights (must be between 13 and 23 meters)
        :param d_2D_out: 2D Distance outdoors between BS and UT
        :return: probability of path being LOS

        NOTE: Distances and heights given in meters
        From Table 7.4.2-1 LOS probability Page 28
        """
        if d_2D_out <= 18:
            # Less than 18 meters
            Pr_LOS = 1
        else:
            if h_ut <= 13:
                # Less than 13 meters
                c_prime = 0
            elif 13 < h_ut <= 23:
                # 13m < h_ut <= 23m
                c_prime = ((h_ut - 13) / 10) ** 1.5
            else:
                printf("User terminal height should be between 13m and 23m for this model")
                exit(0)

            Pr_LOS = ((18/d_2D_out) + (np.exp(-d_2D_out/63) * (1 - 18/d_2D_out))) * \
                     (1 + (c_prime * (5/4) * (d_2D_out/100)**3 * np.exp(-d_2D_out/150)))

        return Pr_LOS

    def pathloss(self, is_los, fc, d_2D, d_3D, h_ut, h_bs):
        """
        LOS/NLOS Pathloss

        :param is_los:
        :param fc:
        :param d_2D:
        :param d_3D:
        :param h_ut:
        :param h_bs:
        :return:

        From Table 7.4.1-1 Pathloss Models Page 26-27
        """
        # pages 26-27 ETSI document
        # TODO: set h_bs to 25, make sure 1.5m ≦ h_ut ≦ 22.5m
        # Freq fc in Hz

        # Compute effective antenna heights (Actual height - effective environment height h_e)
        # h_e is a function of link between BS and UT
        if d_2D <= 18:
            g_2d = 0
        else:
            g_2d = (5/4) * (d_2D/100)**3 * np.exp(-d_2D/150)

        if h_ut < 13:
            # less than 13 meters
            C = 0
        elif 13 <= h_ut <= 23:
            # between 13 and 23 meters
            C = ((h_ut - 13) / 10)**1.5 * g_2d
        else:
            print("Maximum UT heights is 23 meters")
            exit(0)

        # From note1 (UMa)
        if np.random.uniform() > (1 - (1 / (1 + C))):
            # 1 meter
            h_e = 1
        else:
            l_height = list(range(12, int(h_ut-1.5+1), 3))  # +1 to include endpoint
            h_e = np.random.choice(l_height)

        h_bs_eff = h_bs - h_e
        h_ut_eff = h_ut - h_e

        # Propagation velocity in m/s
        c = 3e8

        # Break point distance
        d_bp = 4 * h_bs_eff * h_ut_eff * fc/c

        # LOS (sigma_sf = 4)
        pl = NaN
        if 10 <= d_2D <= d_bp:
            # If distance larger than 10 meters and shorter than breaking point
            pl = 22 * np.log10(d_3D) + 28 + 20 * np.log10(fc)
        elif d_bp < d_2D <= 5000:
            # If distance larger than breaking point and shorter than 5000 meters
            pl = 40 * np.log10(d_3D) + 28 + 20 * np.log10(fc) - 9 * np.log10((d_bp)**2+(h_bs - h_ut)**2)
        else:
            printf("Value of d_2D is outside bounds. Value must be between 10m and 5km")
            exit(0)

        # NLOS (sigma_sf = 6)
        if not is_los:
            #  Pathloss of 3D-UMa LOS outdoor scenario
            pl_3D_UMa_NLOS = 13.54 + 39.08 * np.log10(d_3D) + 20 * np.log10(fc) - 0.6 * (h_ut - 1.5)

            # Pathloss of 3D-UMa LOS outdoor scenario
            pl_3D_UMa_LOS = pl

            pl = max(pl_3D_UMa_NLOS, pl_3D_UMa_LOS)

        return pl

    def chan_model_params(self, isLOS, d_2D, h_ue):
        """
        DS  = rms delay spread
        ASD = rms azimuth spread of departure angles
        ASA = rms azimuth spread of arrival angles
        ZSD = rms zenith spread of departure angles
        ZSA = rms zenith spread of arrival angles
        F = shadow fading
        K = Ricean K-factor

        :return:

        NOTE: fc is carrier frequency in GHz. From Note 6 in Page 41:
        For UMa and frequencies below 6 GHz, use fc = 6 when determining
        the values of the frequency-dependent LSP (large scale parameters) values
        """
        if self.freq//1e9 <= 6:
            # Why? Read NOTE above
            fc = 6
        else:
            fc = self.freq//1e9

        if isLOS:
            chan_params = dict(
            ds_mu_lg=-6.955 - 0.0963 * np.log10(fc),                   # Delay spread (log10(DS/1s)) mean{log10(X)}
            ds_sigma_lg=0.66,                                          # std{log10(X)}
            asd_mu_lg=1.06 + 0.1114 * np.log10(fc),                    # AOD spread (log10(ASD/1°)
            asd_sigma_lg=0.28,
            asa_mu_lg=1.81,                                            # AOA spread (log10(ASA/1°))
            asa_sigma_lg=0.20,
            zsa_mu_lg=0.95,                                            # ZOA spread (log10(ZSA/1°))
            zsa_sigma_lg=0.16,
            zsd_mu_lg=max(-0.5, -2.1*(d_2D/1000)-0.01*(h_ue-1.5)+0.75), # ZOD spread (log10(ZSD/1°)) Tab. 7.5-7 Pp45
            zsd_sigma_lg=0.40,
            zod_offset=0,                                              # In Table 7.5-7, Page 45
            sigma_sf=4,                                                # Shadow fading (SF) [dB]
            k_factor_mu=9,                                             # K-factor (K) [dB]
            k_factor_sigma=3.5,
            num_clusters=12,                                           # Number of clusters N
            rays_per_cluster=20,                                       # Number of rays M per clusters
            delay_scaling=2.5,                                         # Delay scaling parameter r_tau
            per_cluster_shadow_std=3,                                  # Per cluster shadowing std [dB]
            cluster_ds=max(0.25, 6.5622 - 3.4084 * np.log10(fc)),      # Cluster Delay Spread (C_DS) in [ns]
            cluster_asd=5,                                             # Cluster AOD Spread (C_ASD) in [deg]
            cluster_asa=11,                                            # Cluster AOA Spread (C_ASA) in [deg]
            cluster_zsa=7,                                             # Cluster ZOA Spread (C_ZSA) in [deg]
            xpr_mu=8,                                                  # Cross-Polarization Power Ratios [dB]
            xpr_sigma=4
            )
        else:
            chan_params = dict(
            ds_mu_lg=-6.28 - 0.204 * np.log10(fc),                     # Delay spread (log10(DS/1s))
            ds_sigma_lg=0.39,
            asd_mu_lg=1.5 - 0.1144 * np.log10(fc),                     # AOD spread (log10(ASD/1°)
            asd_sigma_lg=0.28,
            asa_mu_lg=2.08 - 0.27 * np.log10(fc),                      # AOA spread (log10(ASA/1°))
            asa_sigma_lg=0.11,
            zsa_mu_lg=-0.3236 * np.log10(fc) + 1.512,                  # ZOA spread (log10(ZSA/1°))
            zsa_sigma_lg=0.16,
            zsd_mu_lg=max(-0.5,-2.1*(d_2D/1000)-0.01*(h_ue-1.5)+0.9),  # ZOD spread (log10(ZSD/1°)) Tab. 7.5-7 Page 45
            zsd_sigma_lg=0.49,                                         # ZOD spread (log10(ZSD/1°)) Tab. 7.5-7 Page 45
            zod_offset=7.66 * np.log10(fc)-5.96 - \
                         10**(0.208*np.log10(fc)-0.782 *
                              np.log10(max(25, d_2D)) -
                              0.13*np.log10(fc)+2.03 -
                              0.07*(h_ue-1.5)),                        # In Table 7.5-7, Page 45 (Note 4: Use fc=6)
            sigma_sf=6,                                                # Shadow fading (SF) [dB]
            k_factor_mu=np.NaN,                                           # K-factor (K) [dB]
            k_factor_sigma=np.NaN,
            num_clusters=20,                                           # Number of clusters N
            rays_per_cluster=20,                                       # Number of rays M per clusters
            delay_scaling=2.3,                                         # Delay scaling parameter r_tau
            per_cluster_shadow_std=3,                                  # Per cluster shadowing std [dB]
            cluster_ds=max(0.25, 6.5622 - 3.4084 * np.log10(fc)),      # Cluster Delay Spread (C_DS) in [ns]
            cluster_asd=5,                                             # Cluster AOD Spread (C_ASD) in [deg]
            cluster_asa=11,                                            # Cluster AOA Spread (C_ASA) in [deg]
            cluster_zsa=7,                                             # Cluster ZOA Spread (C_ZSA) in [deg]
            xpr_mu=8,                                                  # Cross-Polarization Power Ratios [dB]
            xpr_sigma=4
            )

        K = np.random.lognormal(chan_params['k_factor_mu'], chan_params['k_factor_sigma'])
        DS = np.random.lognormal(chan_params['ds_mu_lg'], chan_params['ds_sigma_lg'])
        ASA = np.random.lognormal(chan_params['asa_mu_lg'], chan_params['asa_sigma_lg'])
        ASD = np.random.lognormal(chan_params['asd_mu_lg'], chan_params['asd_sigma_lg'])
        ZSA = np.random.lognormal(chan_params['zsa_mu_lg'], chan_params['zsa_sigma_lg'])
        ZSD = np.random.lognormal(chan_params['zsd_mu_lg'], chan_params['zsd_sigma_lg'])

        # Limit random RMS azimuth arrival and departure spread values to 104 degrees (page 33)
        ASA = min(ASA, 104)
        ASD = min(ASD, 104)
        # Limit random RMS zenith arrival and departure spread values to 52 degrees (page 33)
        ZSA = min(ZSA, 52)
        ZSD = min(ZSD, 52)

        chan_params['K'] = K
        chan_params['DS'] = DS                                                                                          # TODO - compute cross-correlation
        chan_params['ASA'] = ASA
        chan_params['ASD'] = ASD
        chan_params['ZSA'] = ZSA
        chan_params['ZSD'] = ZSD

        return chan_params

    def cluster_delays(self, chan_params, is_los):
        """
        (Step 5)
        :return:
        """
        # Page 33
        # Generate cluster delays (Page 33)

        num_clusters = chan_params['num_clusters']
        k_factor_mu = chan_params['k_factor_mu']
        delay_scaling = chan_params['delay_scaling']
        DS = chan_params['DS']                                                                                          #FIXME - check if linear or log
        K = chan_params['K']

        # Delay spread
        path_delays_temp = -1 * delay_scaling * DS * np.log(np.random.uniform(0, 1, num_clusters))   # (7.5-1)
        path_delays = path_delays_temp - np.min(path_delays_temp)    # Tau_n
        path_delays.sort()
        if self.print_info:
            print("Cluster Delays (in microseconds): {}".format(path_delays * 1e6))

        if is_los:
            # Compensate for effects of LOS peak addition to delay spread
            C_tau =  0.7705 - 0.0433 * K + 0.0002 * K**2 + 0.000017 * K**3       # (7.5-3)
            # Scaled delay below not to be used in cluster power generation
            path_delays_los = path_delays / C_tau                                                              # (7.5-4)
            return path_delays_los

        return path_delays

    def cluster_powers(self, chan_params, path_delays, is_los):
        """
        (Step 6)
        :return:
        """
        per_cluster_shadow_std = chan_params['per_cluster_shadow_std']
        num_clusters = chan_params['num_clusters']
        delay_scaling = chan_params['delay_scaling']
        delay_spread = chan_params['DS']
        k_factor_mu = chan_params['k_factor_mu']
        rays_per_cluster = chan_params['rays_per_cluster']

        # Per cluster shadowing in dB
        Zn = np.random.normal(0.0, per_cluster_shadow_std, num_clusters)
        Pn = np.exp(-path_delays * (delay_scaling - 1) / (delay_scaling * delay_spread)) * 10**(-Zn/10)        # (7.5-5)
        # Normalize powers (sum of all cluster powers == 1)
        Pn_norm = Pn / np.sum(Pn)                                                                              # (7.5-6)

        if is_los:
            # For LOS (include power of single LOS ray)
            k_factor_lin = 10 ** (k_factor_mu / 10)
            P1_los = k_factor_lin / (k_factor_lin + 1)                                                         # (7.5-7)
            Pn = 1 / (k_factor_lin + 1) * Pn_norm                                                              # (7.5-8)
            Pn[0] = Pn[0] + P1_los                                                                      # (7.5-8 Cont'd)
        else:
            Pn = Pn_norm

        # Power of each ray within a cluster
        P_perRay = Pn / rays_per_cluster

        # TODO: Remove clusters with less than -25dB power compared to the maximum cluster power.
        return Pn, P_perRay

    def generate_azimuth_angles(self, type, chan_params, is_los, Pn, azimuth_LOS_degree):
        """
        (Step 7) Generation of AoA and AoD follows same process
        :param type: Angle of Arrival or Angle of Departure. String: "AoD" or "AoA"
        :return:
        """
        K = chan_params['K']
        ASA = chan_params['ASA']
        ASD = chan_params['ASD']
        cluster_asa = chan_params['cluster_asa']
        cluster_asd = chan_params['cluster_asd']
        num_clusters = chan_params['num_clusters']
        rays_per_cluster = chan_params['rays_per_cluster']

        if type == "AoA":
            angle_spread = ASA
            cluster_angle_spread = cluster_asa

        elif type == "AoD":
            angle_spread = ASD
            cluster_angle_spread = cluster_asd

        # Composite PAS (Power Angular Spectrum) in Azimuth
        if is_los:                                                                                            # (7.5-10)
            # From Table 7.5-2 (page 34) with num_clusters equal to 12
            nlos_scaling = 1.146
            # Ricean K-factor in dB
            C_phi = nlos_scaling * (1.1035 - 0.028 * K - 0.002 * K**2 + 0.0001 * K**3)
        else:
            # From Table 7.5-2 (page 34) with num_clusters equal to 20
            nlos_scaling = 1.289
            C_phi = nlos_scaling

        # Clusters' azimuth angle of arrival
        phi_n = (2 * (angle_spread / 1.4) * np.sqrt(-np.log(Pn/np.max(Pn)))) / C_phi                           # (7.5-9)## FIXME - angle spread??? TODO, and which Pn?

        # Assign positive or negative sign to the angles
        X_n = (2 * np.random.randint(2, size=num_clusters) - 1)
        Y_n = np.random.normal(0, angle_spread/7, num_clusters)                                                         # FIXME - check variance!
        if is_los:
            phi_n = (X_n * phi_n + Y_n) - (X_n[0] * phi_n[0] + Y_n[0] - azimuth_LOS_degree)                   # (7.5-12)
        else:
            phi_n = X_n * phi_n + Y_n + azimuth_LOS_degree                                                    # (7.5-11)

        # Add offset angles (from Table 7.5-3) to cluster angles
        ray_offset_angle = [0.0447, 0.1413, 0.2492, 0.3715, 0.5129, 0.6797, 0.8844, 1.1481, 1.5195, 2.1551]
        sign = -1
        phi_n_m = np.empty((num_clusters, rays_per_cluster))
        for n in range(num_clusters):
            for m in range(rays_per_cluster):
                idx = m // 2
                sign *= -1
                phi_n_m[n, m] = phi_n[n] + cluster_angle_spread * (ray_offset_angle[idx] * sign)              # (7.5-13)

        if self.print_info:
            print("Azimuth Angles: {}".format(phi_n_m))

        return phi_n_m

    def generate_zenith_angles(self, type, chan_params, is_los, is_o2i, Pn, zenith_LOS_degree):
        """
        (Step 7) Generation of ZOA assumes that the composite PAS in the zenith dimension of all clusters is Laplacian
        :param type: Angle of Arrival or Angle of Departure. String: "AoD" or "AoA"
        :return:
        """
        K = chan_params['K']
        ZSA = chan_params['ZSA']
        ZSD = chan_params['ZSD']
        cluster_zsa = chan_params['cluster_zsa']
        num_clusters = chan_params['num_clusters']
        rays_per_cluster = chan_params['rays_per_cluster']
        zod_offset = chan_params['zod_offset']
        zsd_mu_lg = chan_params['zsd_mu_lg']

        if type == "AoA":
            angle_spread = ZSA
            cluster_angle_spread = cluster_zsa
        elif type == "AoD":
            angle_spread = ZSD
            cluster_angle_spread = (3/8) * (10**zsd_mu_lg)

        # Composite PAS (Power Angular Spectrum) in Azimuth
        if is_los:                                                                                            # (7.5-10)
            # From Table 7.5-4 (page 35) with num_clusters equal to 12
            nlos_scaling = 1.104
            # Ricean K-factor in dB
            C_theta = nlos_scaling * (1.3086 + 0.0339 * K - 0.0077 * K**2 + 0.0002 * K**3)
        else:
            # From Table 7.5-4 (page 35) with num_clusters equal to 20
            nlos_scaling = 1.178
            C_theta = nlos_scaling

        # Clusters' azimuth angle of arrival
        theta_n = (-angle_spread * np.log(Pn/np.max(Pn))) / C_theta                                           # (7.5-14)## FIXME - angle spread??? TODO, and which Pn?

        # Assign positive or negative sign to the angles
        X_n = (2 * np.random.randint(2, size=num_clusters) - 1)
        Y_n = np.random.normal(0, angle_spread/7, num_clusters)                                                         # FIXME - check variance!
        if is_los:
            theta_n = (X_n * theta_n + Y_n) - (X_n[0] * theta_n[0] + Y_n[0] - zenith_LOS_degree)              # (7.5-17)
        else:
            if type == "AoD":
                theta_n = X_n * theta_n + Y_n + zenith_LOS_degree + zod_offset                                # (7.5-19)
            else:
                if is_o2i:
                    theta_n = X_n * theta_n + Y_n + 90                                                        # (7.5-16)
                else:
                    theta_n = X_n * theta_n + Y_n + zenith_LOS_degree                                         # (7.5-16)

        # Add offset angles (from Table 7.5-3) to cluster angles
        ray_offset_angle = [0.0447, 0.1413, 0.2492, 0.3715, 0.5129, 0.6797, 0.8844, 1.1481, 1.5195, 2.1551]
        sign = -1
        theta_n_m = np.empty((num_clusters, rays_per_cluster))
        for n in range(num_clusters):
            for m in range(rays_per_cluster):
                idx = m // 2
                sign *= -1
                if type == "AoA":
                    theta_n_m[n, m] = theta_n[n] + cluster_angle_spread * (ray_offset_angle[idx] * sign)      # (7.5-18)# FIXME - check theta_n_m falls within [180, 360]
                elif type == "AoD":
                    theta_n_m[n, m] = theta_n[n] + cluster_angle_spread * (ray_offset_angle[idx] * sign)   # (7.5-20)
                if 180 <= theta_n_m[n, m] <= 360:
                    theta_n_m[n, m] = 360 - theta_n_m[n, m]

        if self.print_info:
            print("Zenith Angles: {}".format(theta_n_m))

        return theta_n_m

    def ray_coupling(self):
        """
        (Step 8) Coupling of rays within a cluster for both azimuth and elevation

        :return:
        """
        # FIXME!!!!!!!!!!!!!!!!!

    def generate_cross_polarization(self, chan_params):
        """
        (Step 9) Generate the cross polarization power ratios
        XPR is log-Normal distributed

        :return: xpr_n_m - cross-polarization power ratios for each ray m of each cluster n.
        """
        xpr_mu = chan_params['xpr_mu']
        xpr_sigma = chan_params['xpr_sigma']
        num_clusters = chan_params['num_clusters']
        rays_per_cluster = chan_params['rays_per_cluster']

        X_n_m = np.random.normal(xpr_mu, xpr_sigma, size=(num_clusters, rays_per_cluster))                              # FIXME - is xpr_sigma squared? check all other normal dist.
        xpr_n_m = 10 ** (X_n_m/10)
        return xpr_n_m

    def draw_random_initial_phases(self, chan_params):
        """
        (Step 10) Draw random initial phase for each ray m of each cluster n, and for four different spatial
        polarization combinations (theta/theta, theta/phi, phi/theta, phi/phi). Distribution of initial phases is
        uniform within (-pi, pi)

        :return: phase_init - 3-dimensional array of size (n, m, 4)
        """
        num_clusters = chan_params['num_clusters']
        rays_per_cluster = chan_params['rays_per_cluster']

        # Four polarization combinations
        phase_init = np.random.uniform(-np.pi, np.pi, size=(num_clusters, rays_per_cluster, 4))
        return phase_init

    def channel_coefficients(self, t):
        """
        Generate channel coefficients for each cluster n and each receiver and transmitter element pair
        :return:
        """
        # For each available cell
        for bs in range(self.bs_num):

            # Antenna array
            bs_array = self.array_geometry[bs]
            nS = len(bs_array[:, 0])
            # Single antenna UEs. TODO: currently we are checking every BS-UE pair some will be too distant to consider
            nU = self.ue_num

            # For each transmit and receive element
            for u in range(nU):
                for s in range(nS):

                    # Params
                    chan_params = self.ue_chan_params[bs, u]
                    isLOS = self.ue_isLOS[bs, u]
                    isO2I = 0                                                                                           # FIXME o2i

                    # (a) Power
                    path_delays = self.cluster_delays(chan_params, isLOS)
                    Pn, Pn_perRay = self.cluster_powers(chan_params, path_delays, isLOS)

                    # (c) Coupling between polarizations (rand phases)
                    # phase_init[theta/theta, theta/phi, phi/theta, phi/phi]
                    phase_init = self.draw_random_initial_phases(chan_params)
                    xpr_n_m = self.generate_cross_polarization(chan_params)

                    if isLOS:                                                                              #(see 7.5-29)
                        x00 = np.ones((20, 20))
                        x01 = np.zeros((20, 20))
                        x10 = np.zeros((20, 20))
                        x11 = np.ones((20, 20))
                    else:
                        x00 = np.exp(1j * phase_init[:, :, 0])
                        x01 = np.sqrt(xpr_n_m ** -1) * np.exp(1j * phase_init[:, :, 1])
                        x10 = np.sqrt(xpr_n_m ** -1) * np.exp(1j * phase_init[:, :, 2])
                        x11 = np.exp(1j * phase_init[:, :, 3])

                    pol_coupling_mat = np.empty((2, 2, 20, 20), dtype=np.csingle)
                    pol_coupling_mat[0, 0, :, :] = x00
                    pol_coupling_mat[0, 1, :, :] = x01
                    pol_coupling_mat[1, 0, :, :] = x10
                    pol_coupling_mat[1, 1, :, :] = x11

                    # (e) Array phase offsets
                    # Spherical unit vectors ('n' clusters, 'm' rays within cluster)
                    theta_n_m_zoa = self.generate_zenith_angles("AoA", chan_params, isLOS, isO2I, Pn, self.zoa_los_deg[bs, u])
                    theta_n_m_zod = self.generate_zenith_angles("AoD", chan_params, isLOS, isO2I, Pn, self.zod_los_deg[bs, u])
                    phi_n_m_aoa = self.generate_azimuth_angles("AoA", chan_params, isLOS, Pn, self.aoa_los_deg[bs, u])
                    phi_n_m_aod = self.generate_azimuth_angles("AoD", chan_params, isLOS, Pn, self.aod_los_deg[bs, u])

                    ncluster = chan_params['num_clusters']
                    nrays = chan_params['rays_per_cluster']
                    # rx
                    r_rx_n_m = np.empty((3, ncluster, nrays))                                                 # (7.5-23)
                    r_rx_n_m[0, :, :] = np.sin(theta_n_m_zoa) * np.cos(phi_n_m_aoa)
                    r_rx_n_m[1, :, :] = np.sin(theta_n_m_zoa) * np.sin(phi_n_m_aoa)
                    r_rx_n_m[2, :, :] = np.cos(theta_n_m_zoa)
                    # tx
                    r_tx_n_m = np.empty((3, ncluster, nrays))                                                 # (7.5-24)
                    r_tx_n_m[0, :, :] = np.sin(theta_n_m_zod) * np.cos(phi_n_m_aod)
                    r_tx_n_m[1, :, :] = np.sin(theta_n_m_zod) * np.sin(phi_n_m_aod)
                    r_tx_n_m[2, :, :] = np.cos(theta_n_m_zod)

                    # UE array phase offset
                    # Single antenna UEs so location of antenna is the same as UE location
                    loc_vec_rx_u = self.ue_coord[bs, u]
                    phase_off_ue = np.empty((ncluster, nrays), dtype=np.csingle)
                    for cidx in range(ncluster):
                        for ridx in range(nrays):
                            tmp = np.dot(r_rx_n_m[:, cidx, ridx], loc_vec_rx_u)
                            phase_off_ue[cidx, ridx] = np.exp(1j * 2 * np.pi * 1 / self.wavelength * tmp)

                    # BS array phase offset
                    loc_vec_tx_s = bs_array[s, :]
                    phase_off_bs = np.empty((ncluster, nrays), dtype=np.csingle)
                    for cidx in range(ncluster):
                        for ridx in range(nrays):
                            tmp = np.dot(r_tx_n_m[:, cidx, ridx], loc_vec_tx_s)
                            phase_off_bs[cidx, ridx] = np.exp(1j * 2 * np.pi * 1 / self.wavelength * tmp)

                    # Doppler phase shift (Eq. 7.5-25)
                    v = self.ue_speed[bs, u]
                    phi_v, theta_v = self.ue_direction[bs, u]  # [bs_num, ue_num, [Phi, Theta]]
                    v_vec = v * np.array([np.sin(theta_v) * np.cos(phi_v),
                                          np.sin(theta_v) * np.sin(phi_v),
                                          np.cos(theta_v)])
                    doppler_shift = np.empty((ncluster, nrays), dtype=np.csingle)

                    for cidx in range(ncluster):
                        for ridx in range(nrays):
                            tmp = np.dot(r_rx_n_m[:, cidx, ridx], v_vec)
                            doppler_shift[cidx, ridx] = np.exp(1j * 2 * np.pi * 1 / self.wavelength * tmp * t)

                    # RX antenna pattern (only downlink at the moment, BS=AoD, UE=AoA)
                    # Polarization alternates from one antenna to the next
                    antPolIdx = s % 2
                    F_rx_theta = self.F_ue_theta_gcs[bs, u, antPolIdx, phi_n_m_aoa.astype(int), theta_n_m_zoa.astype(int)]
                    F_rx_phi = self.F_ue_phi_gcs[bs, u, antPolIdx, phi_n_m_aoa.astype(int), theta_n_m_zoa.astype(int)]
                    F_rx = np.array([F_rx_theta, F_rx_phi])
                    # TX antenna pattern
                    F_tx_theta = self.F_bs_theta_gcs[bs, antPolIdx, phi_n_m_aod.astype(int), theta_n_m_zod.astype(int)]
                    F_tx_phi = self.F_bs_phi_gcs[bs, antPolIdx, phi_n_m_aod.astype(int), theta_n_m_zod.astype(int)]
                    F_tx = np.array([F_tx_theta, F_tx_phi])

                    # Sub-Clusters (Table 7.5-5 Page 38)
                    # Sub-Cluster Delays
                    n_sub_cluster = 3
                    sub_path_delays = np.empty((n_sub_cluster, len(path_delays)))
                    for n, tau_n in enumerate(path_delays):
                        sub_path_delays[0, n] = tau_n
                        sub_path_delays[1, n] = tau_n + 1.28 * chan_params['cluster_ds']
                        sub_path_delays[2, n] = tau_n + 2.56 * chan_params['cluster_ds']
                    # Sub-Cluster Mapping (indexes)
                    Ri = []
                    Ri.append(np.array([1, 2, 3, 4, 5, 6, 7, 8, 19, 20]) - 1)
                    Ri.append(np.array([9, 10, 11, 12, 17, 18]) - 1)
                    Ri.append(np.array([13, 14, 15, 16]) - 1)
                    # Sub-Cluster Power
                    p1sc = len(Ri[0]) / chan_params['rays_per_cluster']
                    p2sc = len(Ri[1]) / chan_params['rays_per_cluster']
                    p3sc = len(Ri[2]) / chan_params['rays_per_cluster']

                    # Channel Coefficients
                    # NLOS Channel Matrix (Eq. 7.5-22, Page 37): N-2 weakest clusters (i.e., n = 3, 4, ..., N)
                    phase_offsets = phase_off_ue * phase_off_bs * doppler_shift                               # (7.5-22)
                    H3toN = np.sqrt(Pn_perRay) * np.sum(F_rx * pol_coupling_mat * F_tx * phase_offsets, axis=3)

                    # NLOS Channel Matrix (Eq. 7.5-28, Page 38): Strongest clusters (i.e., n = 1, 2)
                    H1to2 = np.sqrt(Pn_perRay) * F_rx * pol_coupling_mat * F_tx * phase_offsets

                    # Channel impulse response
                    H1 = np.zeros((H3toN.shape[0], H3toN.shape[1]), dtype=complex)
                    H2 = np.zeros((H3toN.shape[0], H3toN.shape[1]), dtype=complex)
                    for tauIdx, tau_n in enumerate(path_delays):

                        # H1
                        # First two clusters
                        for n in range(2):
                            # All three sub-clusters
                            for i in range(n_sub_cluster):
                                # All rays in sub-cluster i
                                for m in Ri[i]:
                                    tau_ni = sub_path_delays[i, n]
                                    if tau_n - tau_ni == 0:
                                        H1 += H1to2[:, :, n, m]

                        # H2
                        for n in range(2, ncluster):

                            H2 += H3toN[:, :, 2::]

                        stop = 1
                        #H = H1to2   + np.sum(H3toN[2::] * )
                        # LOS


                    # Apply pathloss and shadowing



                    """
                    # Construct fdd channel
                    uplink_channel = np.zeros(self.bs_ant_num, dtype=np.complex128)
                    downlink_channel = np.zeros(self.bs_ant_num, dtype=np.complex128)
                    for i in range(path_num):
                        # each path can have 1 to 20 rays
                        if is_los and i == 0:
                            ray_num = 1
                        else:
                            ray_num = np.random.randint(1, 21)
                        ray_elevation_spread = ray_angle_spread  # unit: degree
            
                        # ray_elevations = np.random.normal(
                        #     0.0, ray_elevation_spread, ray_num) / 180 * np.pi
                        ray_elevations = np.random.uniform(
                            -ray_elevation_spread/2, ray_elevation_spread/2, ray_num) /\
                            180 * np.pi
                        ray_azimuth_spread = ray_angle_spread  # unit: degree
                        ray_azimuths = np.random.uniform(
                            -ray_azimuth_spread/2, ray_azimuth_spread/2, ray_num) /\
                            180 * np.pi
                        # ray_azimuths = np.random.normal(
                        #     0.0, ray_azimuth_spread, ray_num) / 180 * np.pi
                        for j in range(ray_num):
                            random_phase = np.exp(1j * 2 * np.pi * np.random.rand(1))
                            uplink_channel += (np.sqrt(path_powers[i] / ray_num)
                                               * random_phase * getRectangArrayRes(
                                               path_elevations[i] + ray_elevations[j],
                                               path_azimuths[i] + ray_azimuths[j],
                                               self.bs_ant_row,
                                               self.bs_ant_col,
                                               self.uplink_ant_space_row_wl_ratio,
                                               self.uplink_ant_space_col_wl_ratio,
                                               False))
                            random_phase = np.exp(1j * 2 * np.pi * np.random.rand(1))
                            downlink_channel += (np.sqrt(path_powers[i] / ray_num)
                                                 * random_phase * getRectangArrayRes(
                                                 path_elevations[i] + ray_elevations[j],
                                                 path_azimuths[i] + ray_azimuths[j],
                                                 self.bs_ant_row,
                                                 self.bs_ant_col,
                                                 self.downlink_ant_space_row_wl_ratio,
                                                 self.downlink_ant_space_col_wl_ratio,
                                                 False))
                    # apply path_loss
                    path_loss = 1
                    uplink_channel = path_loss * uplink_channel
                    downlink_channel = path_loss * downlink_channel
                    return uplink_channel, downlink_channel
                    """
#########################################
#                  Main                 #
#########################################
def main():

    # Channel object
    channel = FDMassiveMIMOChannel()

    # Dumm
    t_vec = range(0, 100)
    for t in t_vec:
        channel.channel_coefficients(t)



    num_ue = 50000
    locAll = np.zeros((num_ue, 3))
    for idx in range(num_ue):
        loc, indoor = channel.place_ue()
        locAll[idx, 0] = loc[0]
        locAll[idx, 1] = loc[1]
        locAll[idx, 2] = loc[2]

    print("MAX: {}".format(np.max(locAll[:, 2])))
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(locAll[:, 0], locAll[:, 1], locAll[:, 2], c='r', marker='o')
    ax.scatter(0, 0, 25, c='b', marker='x')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()

    stop = 1
if __name__ == '__main__':
                                                                                                                      main()