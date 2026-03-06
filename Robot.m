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
            ll;
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
            K;
            phi;
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


        % Inverse kinematics; calculates the control inputs for the
        % concentric tube robot given a current pose and a goal end
        % effector position
        function [rho,theta] = invkinematics(self, q_curr, goal_pos)
            % First we get the rho and theta avlues from q_curr
            rho_curr = get_rho_values(self, q_curr);
            theta_curr = get_theta(self, q_curr);

            % Next, we use rho to get the link lengths
            self.lls = get_links(self, rho_curr);

            % Now we calculate the phi and kappa values
            [self.phi,self.kappa] = calculate_phi_and_kappa(self, theta_curr);

            c = [];
            for i = 1:length(self.lls)
                c(end+1) = self.kappa(i);
                c(end+1) = self.phi(i);
                c(end+1) = self.lls(i);
            end
            c = c';

            % Finally we calculate the base to end effector transform
            T_curr = calculate_transform(self, self.lls, self.phi, self.kappa);

            % Get current end effector position
            p_curr = T_curr(1:3, 4);

            % Calculate (p-p*)
            err = goal_pos - p_curr;

            % Calculate J_p, the Jacobian mapping task space to arc parameters    
            J = jacob0(self, c);
                
            J_omega = J(1:3, :);
            J_v_space = J(4:6, :);
                
            p_skew = [ 0,         -p_curr(3),  p_curr(2);
                       p_curr(3),  0,         -p_curr(1);
                      -p_curr(2),  p_curr(1),  0        ];
                      
            J_p = J_v_space - p_skew * J_omega;
               
            % Modification of equation 12 from paper
            % Calculate change in arc parameters required to move to goal position
            J_pinv = pinv(J_p); 
            delta_c = J_pinv * err;
                
            % Calculate goal arc parameters
            c_goal = c + delta_c;

            % Solve robot-dependent inverse kinematics using an
            % optimization problem (Newton's Method with a central
            % difference gradient)
            rho = rho_curr;
            theta = theta_curr;
            
            max_iter = 100;
            tol = 1e-8;
            
            for i = 1:max_iter               
                % Robot dependent forward kinematics with guess
                [phi_guess, k_guess] = calculate_phi_and_kappa(self, theta);
                ll_guess = get_links(self, rho);

                c_guess = [];
                for i = 1:length(ll_guess)
                    c_guess(end+1) = k_guess(i);
                    c_guess(end+1) = phi_guess(i);
                    c_guess(end+1) = ll_guess(i);
                end
                c_guess = c_guess';
                
                % Residual
                R = c_guess - c_goal;
                
                % Gradient
                G = func_gradient(rho,theta);
                
                % Optimization
                dq = G\-R;
                
                % Update guess
                rho = rho + dq(1:length(rho));
                theta = theta + dq(length(rho)+1:end);
                
                % End condition (error converges)
                if norm(dq) < tol
                    break
                end
            end
        end
        
        % Gradient function for robot-dependent inverse kinematics
        % optimization problem
        function J = func_gradient(rho,theta)

            h = 1e-6;
            J = [];
            
            [phi, k] = calculate_phi_and_kappa(self, theta);
            ll = get_links(self, rho);

            for i = 1:length(rho)
                rho_ph = rho;
                rho_mh = rho;
                rho_ph(i) = rho_ph(i) + h;
                rho_mh(i) = rho_mh(i) - h;
                ll_ph = get_links(self, rho_ph);
                ll_mh = get_links(self, rho_mh);
                
                c_ph = [];
                for i = 1:length(ll_ph)
                    c_ph(end+1) = k(i);
                    c_ph(end+1) = phi(i);
                    c_ph(end+1) = ll_ph(i);
                end
                c_ph = c_ph';

                c_mh = [];
                for i = 1:length(ll_mh)
                    c_mh(end+1) = k(i);
                    c_mh(end+1) = phi(i);
                    c_mh(end+1) = ll_mh(i);
                end
                c_mh = c_mh';

                J(:, end+1) = (c_ph-c_mh)/(2*h);
            end

            for i = 1:length(theta)
                theta_ph = theta;
                theta_mh = theta;
                theta_ph(i) = theta_ph(i) + h;
                theta_mh(i) = theta_mh(i) - h;
                [phi_ph, k_ph] = calculate_phi_and_kappa(self, theta_ph);
                [phi_mh, k_mh] = calculate_phi_and_kappa(self, theta_mh);

                c_ph = [];
                for i = 1:length(ll_ph)
                    c_ph(end+1) = k_ph(i);
                    c_ph(end+1) = phi_ph(i);
                    c_ph(end+1) = ll(i);
                end
                c_ph = c_ph';

                c_mh = [];
                for i = 1:length(ll_mh)
                    c_mh(end+1) = k_mh(i);
                    c_mh(end+1) = phi_mh(i);
                    c_mh(end+1) = ll(i);
                end
                c_mh = c_mh';

                J(:, end+1) = (c_ph-c_mh)/(2*h);
            end
        end

        
        % Robot independent continuum Jacobian solution recycled from HW 4
        function J = jacob0(self, c)
            J = [];
            ll = [];
            dphi = [];
            K = [];
            for i = 1:length(c)/3
                K(end+1) = c(3*(i-1)+1);
                dphi(end+1) = c(3*(i-1)+2);
                ll(end+1) = c(3*(i-1)+3);
            end
            
            for i = 1:length(c)/3
                k = K(i);
                phi = dphi(i);
                l = ll(i);
                Jnew = [-l*sin(phi), 0, -k*sin(phi);
                          l*cos(phi), 0, k*cos(phi);
                         0, 1, 0;
                         cos(phi)*(cos(k*l)-1)/(k^2), 0, 0;
                         sin(phi)*(cos(k*l)-1)/(k^2), 0, 0;
                         -(sin(k*l)-k*l)/(k^2), 0, 1];
                J(:,end+1:end+3) = Jnew;
            end
            
            for i = 2:length(c)/3
               T = calculate_transform(self, ll(1:i-1), dphi(1:i-1), K(1:i-1));
               R = T(1:3,1:3);
               p = T(1:3,4);
               p_square = [0, -p(3), p(2);
                           p(3), 0, -p(1);
                           -p(2), p(1), 0];
               AdT = [R, zeros(3); p_square * R, R];
                
               for j = 1:3
                  J(:,3*(i-1)+j) = AdT*J(:,3*(i-1)+j);
               end
            end
        end
    end
end

