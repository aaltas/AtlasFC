classdef mav_viewer < handle
% MAV_VIEWER  3D aircraft visualization for AtlasFC.
%
%   Renders a simplified fixed-wing UAV in 3D space and updates its
%   position and attitude in real time. The aircraft geometry is defined
%   in the body frame and transformed to inertial (NED→ENU for MATLAB plot).
%
%   USAGE:
%     viewer = mav_viewer();
%     viewer.update(pn, pe, h, phi, theta, psi);
%
%   COORDINATE CONVENTION:
%     Input is in NED (North-East-Down):
%       pn = north [m], pe = east [m], h = altitude [m] (positive up)
%     MATLAB 3D plot uses ENU so we apply NED→ENU transform internally.
%
%   Based on Beard & McLain, Ch.2, Project Aircraft (Slides 24-26).

    properties
        fig_handle
        body_handle
        Vertices        % aircraft vertices in body frame [3 x N]
        Faces           % face index list
        FaceColors      % color per face
        initialized     % bool
    end

    methods
        %------------------------------------------------------------------
        function self = mav_viewer()
            self.initialized = false;
            [self.Vertices, self.Faces, self.FaceColors] = self.define_aircraft();
        end

        %------------------------------------------------------------------
        function update(self, pn, pe, h, phi, theta, psi)
        % UPDATE  Redraw the aircraft at given position and attitude.
        %
        %   update(pn, pe, h, phi, theta, psi)
        %
        %   INPUTS:
        %     pn, pe  [m]   - north, east position (NED)
        %     h       [m]   - altitude (positive up, h = -pd)
        %     phi     [rad] - roll
        %     theta   [rad] - pitch
        %     psi     [rad] - yaw

            % --- 1. Rotate body vertices to inertial frame ---
            R_bi = euler_to_rotation(phi, theta, psi)';  % body → inertial
            V_ned = R_bi * self.Vertices;                 % [3 x N] in NED

            % --- 2. Translate to position ---
            V_ned(1,:) = V_ned(1,:) + pn;
            V_ned(2,:) = V_ned(2,:) + pe;
            V_ned(3,:) = V_ned(3,:) - h;   % NED down = negative altitude

            % --- 3. Convert NED → ENU for MATLAB plot ---
            % MATLAB's 3D axes: x=East, y=North, z=Up
            % NED:  x=North, y=East,  z=Down
            R_ned2enu = [0, 1, 0; 1, 0, 0; 0, 0, -1];
            V_plot = R_ned2enu * V_ned;

            % --- 4. Draw or update ---
            if ~self.initialized
                self.fig_handle = figure('Name','AtlasFC - MAV Viewer','NumberTitle','off');
                self.body_handle = patch( ...
                    'Vertices',         V_plot', ...
                    'Faces',            self.Faces, ...
                    'FaceVertexCData',  self.FaceColors, ...
                    'FaceColor',        'flat', ...
                    'EdgeColor',        'k', ...
                    'LineWidth',        0.5);
                xlabel('East [m]');
                ylabel('North [m]');
                zlabel('Altitude [m]');
                title('AtlasFC — MAV Viewer (Ch.2)');
                view(45, 25);
                axis equal;
                grid on;
                hold on;
                self.initialized = true;
            else
                set(self.body_handle, 'Vertices', V_plot');
                drawnow limitrate;
            end
        end

        %------------------------------------------------------------------
        function [V, F, C] = define_aircraft(~)
        % DEFINE_AIRCRAFT  Define UAV geometry in body frame.
        %
        %   Body frame convention (NED):
        %     +x = nose (forward)
        %     +y = right wing
        %     +z = down (belly)
        %
        %   Vertices are defined as 3D points in body frame [m].
        %   Dimensions are inspired by a small fixed-wing UAV (~1m wingspan).

            % --- Fuselage dimensions ---
            fuse_l1    =  0.30;   % nose to wing leading edge
            fuse_l2    =  0.10;   % wing root chord
            fuse_l3    =  0.60;   % wing to tail
            fuse_h     =  0.08;   % fuselage height
            fuse_w     =  0.06;   % fuselage width

            % --- Wing dimensions ---
            wing_l     =  0.20;   % chord length
            wing_w     =  0.50;   % half-span (total span = 2*wing_w)

            % --- Tail dimensions ---
            tailwing_l =  0.10;
            tailwing_w =  0.20;
            tail_h     =  0.15;

            % ---- Vertices (in body frame: x-fwd, y-right, z-down) ----
            % Fuselage — 8 corner vertices forming a box
            V = [...
                % Nose cone (point)
                fuse_l1,           0,          0;  ... % 1  nose tip
                % Fuselage front section
                0,                 fuse_w/2,   fuse_h/2;  ... % 2
                0,                -fuse_w/2,   fuse_h/2;  ... % 3
                0,                -fuse_w/2,  -fuse_h/2;  ... % 4
                0,                 fuse_w/2,  -fuse_h/2;  ... % 5
                % Fuselage rear (at tail)
               -fuse_l3,           fuse_w/2,   fuse_h/4;  ... % 6
               -fuse_l3,          -fuse_w/2,   fuse_h/4;  ... % 7
               -fuse_l3,          -fuse_w/2,  -fuse_h/4;  ... % 8
               -fuse_l3,           fuse_w/2,  -fuse_h/4;  ... % 9
                % Main wings (right)
                0,                 wing_w,     0;  ... % 10 right tip LE
               -wing_l,            wing_w,     0;  ... % 11 right tip TE
               -wing_l,            fuse_w/2,   0;  ... % 12 right root TE
                0,                 fuse_w/2,   0;  ... % 13 right root LE
                % Main wings (left)
                0,                -wing_w,     0;  ... % 14 left tip LE
               -wing_l,           -wing_w,     0;  ... % 15 left tip TE
               -wing_l,           -fuse_w/2,   0;  ... % 16 left root TE
                0,                -fuse_w/2,   0;  ... % 17 left root LE
                % Horizontal tail (right)
               -fuse_l3,           tailwing_w, 0;  ... % 18
               -fuse_l3-tailwing_l, tailwing_w, 0; ... % 19
               -fuse_l3-tailwing_l, fuse_w/2,   0; ... % 20
                % Horizontal tail (left)
               -fuse_l3,          -tailwing_w, 0;  ... % 21
               -fuse_l3-tailwing_l,-tailwing_w, 0; ... % 22
               -fuse_l3-tailwing_l,-fuse_w/2,   0; ... % 23
                % Vertical tail
               -fuse_l3,           0,           0; ... % 24
               -fuse_l3,           0,          -tail_h; ... % 25
               -fuse_l3-tailwing_l, 0,          -tail_h; ... % 26
               -fuse_l3-tailwing_l, 0,           0; ... % 27
            ]';  % transpose → [3 x N]

            % ---- Faces (vertex index lists) ----
            F = [...
                1,  2,  3,  3;  ... % fuselage top-right
                1,  3,  4,  4;  ... % fuselage top-left
                1,  4,  5,  5;  ... % fuselage bottom-left
                1,  5,  2,  2;  ... % fuselage bottom-right
                2,  3,  7,  6;  ... % fuselage side top
                3,  4,  8,  7;  ... % fuselage side left
                4,  5,  9,  8;  ... % fuselage side bottom
                5,  2,  6,  9;  ... % fuselage side right
                6,  7,  8,  9;  ... % fuselage tail end
                10, 11, 12, 13; ... % right wing
                14, 15, 16, 17; ... % left wing
                18, 19, 20, 6;  ... % horizontal tail right
                21, 22, 23, 7;  ... % horizontal tail left
                24, 25, 26, 27; ... % vertical tail
            ];

            % ---- Colors per face ----
            gray    = [0.6, 0.6, 0.6];
            dk_gray = [0.3, 0.3, 0.3];
            blue    = [0.2, 0.4, 0.8];
            red     = [0.8, 0.2, 0.2];
            yellow  = [0.9, 0.8, 0.1];

            C = [...
                gray;    % fuse top-right
                gray;    % fuse top-left
                dk_gray; % fuse bottom-left
                dk_gray; % fuse bottom-right
                gray;    % side top
                gray;    % side left
                dk_gray; % side bottom
                dk_gray; % side right
                red;     % tail end
                blue;    % right wing
                blue;    % left wing
                yellow;  % h-tail right
                yellow;  % h-tail left
                gray;    % vertical tail
            ];
        end
    end
end
