% =========================================================================
%  MAG_MODEL - Magnetometer heading sensor model
% =========================================================================
%  Models a 3-axis magnetometer used as a compass (heading reference).
%  The simplified B&M Ch.7 model outputs a heading angle (psi) rather
%  than a raw magnetic field vector, since the full vector model requires
%  magnetic declination calibration.
%
%  Measurement equation (simplified, zero magnetic declination):
%    m_body = R_bv * [cos(decl); sin(decl); 0]   (unit mag field vector)
%
%  For zero declination (decl = 0):
%    m_body = R_bv * [1; 0; 0]
%
%  Heading extracted from body-frame measurements:
%    y_mag = atan2(m_body(2), m_body(1))  + noise
%          = psi + noise  (for level flight, phi≈0, theta≈0)
%
%  This model outputs a heading angle measurement directly.
%  The result is wrapped to (-pi, pi].
%
%  Inputs:
%    x13       - 13-state vector [pn,pe,pd,u,v,w,e0,e1,e2,e3,p,q,r]
%    sparams   - sensor_params() struct
%
%  Outputs:
%    y_mag     - [scalar] measured heading angle [rad], wrapped to (-pi, pi]
%
%  Usage:
%    y_mag = mag_model(x13, sparams);
%
%  Author : AtlasFC
%  Ref    : Beard & McLain, "Small Unmanned Aircraft", Ch. 7
% =========================================================================

function y_mag = mag_model(x13, sparams)

    % Rotation matrix: body frame ← vehicle/NED frame
    R_bv = quaternion_to_rotation(x13(7:10));

    % Magnetic field vector in NED (unit vector, zero declination)
    m_ned = [1; 0; 0];

    % Rotate to body frame
    m_body = R_bv * m_ned;

    % Extract heading from body-frame magnetometer readings
    % psi_mag = atan2(m_y_body, m_x_body)  (only valid for small roll/pitch)
    psi_true = atan2(m_body(2), m_body(1));

    % Add white noise
    noise  = sparams.sigma_mag * randn;
    y_mag  = psi_true + noise;

    % Wrap to (-pi, pi]
    y_mag  = atan2(sin(y_mag), cos(y_mag));

end
