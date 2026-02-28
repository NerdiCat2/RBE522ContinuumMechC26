classdef Robot < handle

    properties
        % Save vector of tubes and size of vector
        tubes = []
        num_tubes = 0

        % Save link lengths, phi values, and kappa values vectors (1 x num
        % links)
        lls = []
        phi = []
        kappa = []
    end

    methods
        % Constructor. This creates an instance of the robot class 
        % Must pass in a vector of tubes
        function self = Robot(tubes)
            self.tubes = tubes;
            self.num_tubes = size(tubes, 2);
        end

        % Here we calculate the kinematics of a full CTR
        % Pass in the raw joint variables
        % Return transformation matrix that goes from home frame to end
        % effector frame
        % See functions below for each step
        function T = fkin(self, q_var)  
            % First we get the rho and theta avlues from q_var
            rho = get_rho_values(self, q_var);
            theta = get_theta(self, q_var);

            % Next, we use rho to get the link lengths
            self.lls = get_links(self, rho);

            % Now we calculate the phi and kappa values
            [self.phi,self.kappa] = calculate_phi_and_kappa(self, theta);

            % Finally we calculate the base to end effector transform
            T = calculate_transform(self, self.lls, self.phi, self.kappa);
        end

        % Get rho values from joint positions
        % Return rho (1 x i vector, i is num tubes)
        function rho = get_rho_values(self, q_var)
            % Here we extract rho values from the q_var vector

            % Initialize a vector to hold the result
            rho = zeros([1 self.num_tubes]);

            for i=2:self.num_tubes
                rho(i) = (q_var(i) - q_var(1)) * 10^-3;
            end

        end

        % Function to find the link lengths, in order
        % Returns link lengths (1 x j vector, where j is num links)
        function ll = get_links(self, rho)
            d = [];
            for i = 1:length(self.tubes)
                d(end+1) = self.tubes(i).d;
            end
            transition_points = [];
            for i = 1:length(d)
                transition_points(end+1) = rho(i);
                transition_points(end+1) = rho(i) + d(i);
            end
            transition_points = sort(transition_points);
            ll = [];
            for i = 1:length(transition_points)-1
                ll(end+1) = transition_points(i+1)-transition_points(i);
            end
            ll
        end

        % Function to get theta values
        % Returns theta (1 x j vector where j is num links)
        function theta = get_theta(self, q_var)
            % Here we extract theta values from the q_var vector

            % Initialize a vector to hold the result
            theta = zeros([1 self.num_tubes]);

            for i=1:self.num_tubes
                theta(i) = deg2rad(q_var(i+self.num_tubes));
            end
        end


        % Function to calcualte phi values for two or three tube
        % configurations
        % Should return phi (1 x j vector, where j is num links)
        % and K (1 x j vector)
        function [phi,K] = calculate_phi_and_kappa(self, theta)
            % Calculate second moments of area
            K = [];
            phi = [];
            k = zeros(length(self.tubes));
            % for i = 1:length(self.tubes)
            %     k(i, i) = self.tubes(i).k;
            %     k(i, ?) = -1;
            % end
            if length(self.tubes) == 2
                k = [self.tubes(1).k, self.tubes(1).k, -1;
                     0, self.tubes(2).k, self.tubes(2).k];
            elseif length(self.tubes) == 3
                k = [self.tubes(1).k, self.tubes(1).k, -1, -1, -1;
                     0, self.tubes(2).k, self.tubes(2).k, self.tubes(2).k, -1;
                     0, 0, 0, self.tubes(3).k, self.tubes(3).k];
            end

            for j = 1:length(self.lls)
                chi_num = 0;
                chi_den = 0;
                gamma_num = 0;
                gamma_den = 0;
                for i = 1:length(self.tubes)
                    I = self.tubes(i).I;
                    E = self.tubes(i).E;
                    ki = k(i, j);
                    if ki == -1
                        E = 0;
                    end
                    chi_num = chi_num + E*I*ki*cos(theta(i));
                    chi_den = chi_den + E*I;
                    gamma_num = gamma_num + E*I*ki*sin(theta(i));
                    gamma_den = gamma_den + E*I;
                end
                chi = chi_num/chi_den;
                gamma = gamma_num/gamma_den;
                K(j) = sqrt(chi^2 + gamma^2);
                phi(j) = atan2(gamma,chi);
            end
            
            % Print results
            K
            phi
        end

        % Take in all robot dependent parameters (lls, phi, kappa) and
        % compelte the robot independent constant curvature kinamtatics
        % Returns a 4x4 transformation matrix from base frame to end
        % effector
        function T = calculate_transform(self, ll, phi, K)
            T = eye(4);

            dphi = phi;
            for i = 2:length(phi)
               dphi(i) = phi(i)-phi(i-1); 
            end
            
            for ii = 1 : length(ll)
                Tii = self.arckinematics(K(ii), dphi(ii), ll(ii));
                T = T * Tii;
            end
        end


        function T = arckinematics(self, K, dphi, ll)
            w_rot = [0, -1, 0;
                     1, 0, 0;
                     0, 0, 0];
            v_rot = [0, 0, 0]';
            exp_rot_1 = eye(3)+sin(dphi)*w_rot + (1-cos(dphi))*w_rot^2;
            exp_rot_2 = (eye(3)*dphi + (1-cos(dphi))*w_rot + (dphi-sin(dphi))*w_rot^2)*v_rot;
            exp_rot = [exp_rot_1, exp_rot_2;
                       0, 0, 0, 1];

            w_tran = [0, 0, 0;
                     0, 0, -K;
                     0, K, 0];
            v_tran = [0, 0, 1]';
            exp_tran_1 = eye(3)+sin(ll)*w_tran + (1-cos(ll))*w_tran^2;
            exp_tran_2 = (eye(3)*ll + (1-cos(ll))*w_tran + (ll-sin(ll))*w_tran^2)*v_tran;
            exp_tran = [exp_tran_1, exp_tran_2;
                       0, 0, 0, 1];
            T = exp_rot*exp_tran;
        end
    end
end

