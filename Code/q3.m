%----------------------------------------
% MTE 204 - Project 2
% Project 2: 2D Truss
% Question: 3
%----------------------------------------
format long; % Set the display format to show more digits
%----------------------------------------
% Problem 3: 2D Truss
%----------------------------------------

% Given Parameters
Eg = 200e9;                % Young's Modulus (Pa)
Ag = 4e-4;                 % Cross-sectional Area (m^2)
Lg = 1.2;                  % Length of truss element (m)

% Support Conditions (Constrained DOFs)
% Assuming Node 1 is fixed in both x and y, Node 5 is fixed in y
removeDOFs = [2, 10];   % DOF numbers to remove (constraints)

% External Forces
F = zeros(10, 1);          % Initialize force vector (10 DOFs for 5 nodes)
F(4) = -2000;              % Force at DOF 4 (y-direction at Node 2)
F(8) = -5000;              % Force at DOF 8 (y-direction at Node 4)

% Truss Elements: [node1, node2, theta]
elements = [
    1, 2, 60;   % Element 1
    1, 3, 0;    % Element 2
    2, 3, -60;  % Element 3
    3, 4, 60;   % Element 4
    3, 5, 0;    % Element 5
    4, 5, -60;  % Element 6
    2, 4, 0;    % Element 7
];

%----------------------------------------
% Global Stiffness Matrix Assembly
%----------------------------------------

numNodes = 5;
numDOFs = 2 * numNodes;   % Total DOFs
K_global = zeros(numDOFs, numDOFs); % Initialize global stiffness matrix

% Loop through each element and add stiffness contributions
for i = 1:size(elements, 1)
    node1 = elements(i, 1);
    node2 = elements(i, 2);
    theta = elements(i, 3);
    % Generate element stiffness matrix
    mat = generateGlobalStiffness(theta, node1, node2, Ag, Eg, Lg, numNodes);
    fprintf('Global Stiffness Matrix for Element %d:\n', i);
    disp(mat);
    K_global = K_global + mat;
    % K_global = K_global + generateGlobalStiffness(theta, node1, node2, Ag, Eg, Lg, numNodes);
end

%----------------------------------------
% Apply Boundary Conditions
%----------------------------------------

% Remove constrained DOFs from the global stiffness matrix and force vector
F_reduced = F;
K_reduced = K_global;

% Remove rows and columns corresponding to constrained DOFs
K_reduced(removeDOFs, :) = [];
K_reduced(:, removeDOFs) = [];
F_reduced(removeDOFs) = [];

%----------------------------------------
% Solve System of Equations
%----------------------------------------

% Solve for the displacements of free nodes
% The mldivide was chosen as this method automatically selects the appropriate solver based on the matrix properties
% and therefore this will find the most effective method of solving the equation.
XG = K_reduced \ F_reduced;

% Reconstruct full displacement vector including zeros for removed DOFs
displacements = zeros(numDOFs, 1);
freeDOFs = setdiff(1:numDOFs, removeDOFs);
displacements(freeDOFs) = XG;

%----------------------------------------
% Display Global Displacement Vector
%----------------------------------------

disp('Global Displacement Vector (m):');
for i = 1:numDOFs
    fprintf('DOF %d: %.6e m\n', i, displacements(i));
end

%----------------------------------------
% Compute Element Stresses
%----------------------------------------

normal_stresses = zeros(size(elements, 1), 1); % Initialize stress array

for i = 1:size(elements, 1)
    node1 = elements(i, 1);
    node2 = elements(i, 2);
    theta = elements(i, 3);

    % Direction cosines
    c = cosd(theta);
    s = sind(theta);

    % DOFs for nodes
    dof1 = [2*node1-1, 2*node1]; % Node 1 DOFs
    dof2 = [2*node2-1, 2*node2]; % Node 2 DOFs

    % Axial deformation
    delta_L = c * (displacements(dof2(1)) - displacements(dof1(1))) + ...
              s * (displacements(dof2(2)) - displacements(dof1(2)));

    % Normal stress
    normal_stresses(i) = (Eg / Lg) * delta_L;
end

%----------------------------------------
% Display Results
%----------------------------------------

% Node displacements (x and y directions)
disp('Node Displacements (m):');
displacementsXY = reshape(displacements, 2, numNodes)';
for i = 1:numNodes
    fprintf('Node %d: ux = %.6e m, uy = %.6e m\n', i, displacementsXY(i, 1), displacementsXY(i, 2));
end

% Element stresses
disp('Normal Stresses in Each Member (Pa):');
for i = 1:size(elements, 1)
    fprintf('Element %d: Normal Stress = %.6e Pa\n', i, normal_stresses(i));
end

%----------------------------------------
% Supporting Functions
%----------------------------------------

function K_global_internal = generateGlobalStiffness(thetaDeg, node1, node2, A, E, L, numNodes)
    % Generate global stiffness matrix for a truss element
    c = cosd(thetaDeg);
    s = sind(thetaDeg);
    k_local = (E * A / L) * [
        c^2,  c*s, -c^2, -c*s;
        c*s,  s^2, -c*s, -s^2;
       -c^2, -c*s,  c^2,  c*s;
       -c*s, -s^2,  c*s,  s^2;
    ];
    dof1 = [2*node1-1, 2*node1];
    dof2 = [2*node2-1, 2*node2];
    dof = [dof1, dof2];
    K_global_internal = zeros(2*numNodes, 2*numNodes);
    K_global_internal(dof, dof) = K_global_internal(dof, dof) + k_local;
end
