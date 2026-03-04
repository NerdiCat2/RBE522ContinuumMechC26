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
            [self.phi,self.kappa] = calculate_phi_and_kappa(self, theta,rho);

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
        function s = get_links(self, rho)
            d=[];
            for i = 1:length(self.tubes)
                d(end+1) = self.tubes(i).d;
            end
            transition_points = [rho, rho + d];
            T_sorted = sort(transition_points);
            s = diff(T_sorted);
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
        function [phi,K] = calculate_phi_and_kappa(self, theta,rho)
            numLinks=self.num_tubes*2-1;
            OD=[];
            ID=[];
            d=[];
            K=[];
            I=[];
            E=[];
            k=[];
            for i = 1:length(self.tubes)
                OD(end+1) = self.tubes(i).od;
                ID(end+1) = self.tubes(i).id;
                d(end+1) = self.tubes(i).d;
                I(end+1) = self.tubes(i).I;
                E(end+1) = self.tubes(i).E;
                k(end+1) = self.tubes(i).k;
            end
            T=sort([d+rho rho]);
            phiAbs = zeros(1,numLinks);
            tubeEnds = rho+d;
            ll=self.lls;
            for i = [1:self.num_tubes]
                mid = T(i) + ll(i)/2;
                active = (mid > rho) & (mid < tubeEnds);
                present = (mid < tubeEnds);
                
                if active==zeros(self.num_tubes)
                    % a=zeros(1,self.num_tubes);
                    % b=ones(1,self.num_tubes);
                    % theta=a;
                    k(i)=0;
                    phiAbs(i)=0;

                else
                    Ea=zeros(size(E))
                    Ia=zeros(size(I))
                    Ka=zeros(size(k))
                    Ip=zeros(size(I))
                    Thetaa=zeros(size(theta))
                    for i=1:size(active,2)
                        if active(i)
                            Ea(i)=E(i)
                            Ia(i)=I(i)
                            Ka(i)=k(i)
                            Thetaa(i)=theta(i)
                        end
                        if present(i)
                            Ip(i)=I(i)
                        end
                    end
                    a=Ea.*Ia.*Ka
                    b=sum(Ea.*Ip)
                    chi=sum(a.*cos(Thetaa))/b
                    gamma=sum(a.*sin(Thetaa))/b
                    k(i) = sqrt(chi^2 + gamma^2)
                    phiAbs(i) = atan2(gamma, chi)
                end
            end
            phi = diff([phiAbs])
            K=k;
        end

        % Take in all robot dependent parameters (lls, phi, kappa) and
        % compelte the robot independent constant curvature kinamtatics
        % Returns a 4x4 transformation matrix from base frame to end
        % effector
        function T = calculate_transform(self, s, phi, K)
            T=[eye(4)];
            % ********************************************************
            %                          TODO
            % ********************************************************
        end
    end
end

