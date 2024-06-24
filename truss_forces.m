function truss_forces()
    % Given parameters
    mass_truck = 3500; % kg
    g = 9.81; % m/s^2, acceleration due to gravity
    W_truck = mass_truck * g; % weight of the truck in N
    d = input('Enter the value of d (m): ');
    % Lengths of members (in meters)
    L1 = 3.6; % Length of member AB
    L2 = 3.6; % Length of member BC
    L3 = 3.6; % Length of member CD
    L4 = 3.6; % Length of member DE
    L5 = 2.0; % Length of member EF
    L6 = 3.0; % Length of member FG
    L7 = 3.0; % Length of member GH
    L8 = 2.0; % Length of member HA
    L9 = 10.0; % Length of member AE
    L10 = 4.2; % Length of member DG
    L11 = 4.2; % Length of member BG

    % Angles (in degrees)
    theta1 = atan(3/2); % Angle for AB
    theta2 = atan(3/2); % Angle for BC
    theta3 = atan(3/3); % Angle for DE
    theta4 = atan(3/2); % Angle for CD
    theta5 = atan(3/2); % Angle for EF
    theta6 = atan(2/3); % Angle for DG
    theta7 = atan(2/3); % Angle for BG

    theta1 = rad2deg(theta1);
    theta2 = rad2deg(theta2);
    theta3 = rad2deg(theta3);
    theta4 = rad2deg(theta4);
    theta5 = rad2deg(theta5);
    theta6 = rad2deg(theta6);
    theta7 = rad2deg(theta7);

    % Calculate reaction forces at supports
    syms RA RE
    equations = [RA + RE == W_truck, ...
                 RA * 10 - W_truck * d == 0];
    S = solve(equations, [RA, RE]);
    RA = double(S.RA);
    RE = double(S.RE);

    % Joint A
    FAB = RA / sind(theta1);
    FAH = -FAB * cosd(theta1);

    % Joint B
    syms FBC FBG
    equations = [FBC * sind(theta4) + FBG * cosd(theta2) == 0, ...
                 FAB + FBC * cosd(theta4) - FBG * sind(theta2) == 0];
    S = solve(equations, [FBC, FBG]);
    FBC = double(S.FBC);
    FBG = double(S.FBG);

    % Joint C
    FCG = -FBC * sind(theta4);

    % Joint D
    FCD = FBC; % due to symmetry
    FDE = RE / sind(theta3);
    FEF = -FDE * cosd(theta3);

    % Joint G
    FDG = FBG; % due to symmetry

    % Output results
    fprintf('For d = %.2f meters:\n', d);
    fprintf('RA = %.2f N\n', RA);
    fprintf('RE = %.2f N\n', RE);
    fprintf('Forces in members:\n');
    fprintf('F_AB = %.2f N\n', FAB);
    fprintf('F_BC = %.2f N\n', FBC);
    fprintf('F_BG = %.2f N\n', FBG);
    fprintf('F_CG = %.2f N\n', FCG);
    fprintf('F_CD = %.2f N\n', FCD);
    fprintf('F_DG = %.2f N\n', FDG);
    fprintf('F_DE = %.2f N\n', FDE);
    fprintf('F_EF = %.2f N\n', FEF);

    % Plot forces in members CG, BG, and BC as a function of d
    d_vals = 0:0.1:10; % from 0 to 10 meters
    F_CG_vals = zeros(size(d_vals));
    F_BG_vals = zeros(size(d_vals));
    F_BC_vals = zeros(size(d_vals));

    for i = 1:length(d_vals)
        di = d_vals(i);

        % Recalculate RA and RE for each d
        syms RA RE
        equations = [RA + RE == W_truck, ...
                     RA * 10 - W_truck * di == 0];
        S = solve(equations, [RA, RE]);
        RA = double(S.RA);
        RE = double(S.RE);

        % Recalculate forces for each d
        FAB = RA / sind(theta1);

        % Joint B
        syms FBC FBG
        equations = [FBC * sind(theta4) + FBG * cosd(theta2) == 0, ...
                     FAB + FBC * cosd(theta4) - FBG * sind(theta2) == 0];
        S = solve(equations, [FBC, FBG]);
        FBC = double(S.FBC);
        FBG = double(S.FBG);

        % Joint C
        FCG = -FBC * sind(theta4);

        % Store the values
        F_CG_vals(i) = FCG;
        F_BG_vals(i) = FBG;
        F_BC_vals(i) = FBC;
    end

    % Plotting
    figure;
    plot(d_vals, F_CG_vals, 'r', 'DisplayName', 'F_CG');
    hold on;
    plot(d_vals, F_BG_vals, 'g', 'DisplayName', 'F_BG');
    plot(d_vals, F_BC_vals, 'b', 'DisplayName', 'F_BC');
    xlabel('Distance d (m)');
    ylabel('Force (N)');
    title('Forces in Members CG, BG, and BC as a Function of d');
    legend;
    grid on;
end
