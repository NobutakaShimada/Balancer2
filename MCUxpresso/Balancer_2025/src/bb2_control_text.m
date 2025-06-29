% ������
clear;clc;

% �p�����[�^
Rw = 0.021; % �ԗ֔��a
l = 0.135; % �d�S����
M = 0.21; % �{�̏d��
m = 0.0053; % �ԗ֏d��
Jb = M*l^2; % �{�̊������[�����g
Jw = m*Rw^2/2.0; % �ԗ֊������[�����g
fm = 4.91e-7; % ���[�^���S�����C�W��
g = 9.81; % �d�͉����x
n = 21; % �M�A��
Kt = 1.15e-3; % ���[�^�g���N�W��

% ��ԕ������̋L�q
E = [(m + M)*Rw^2 + Jw  M*Rw*l;
     M*Rw*l             M*l*l + Jb];
F = [2*fm  -2*fm;
    -2*fm  2*fm];
G = [0  0;
     0  -M*g*l];
H = [-2*n*Kt; 2*Kt];
A = [zeros(2)  eye(2);
     -E\G  -E\F];
B = [zeros(2, 1); E\H];
C = eye(4);

% A�s��̌ŗL�l���m�F
EigA = eig(A);

% �Q�C���ݒ�(�ǂ��炩��p����)
% ----------------------------------
% �ɔz�u
a1=-5;
a2=-10;
b1=3;
b2=5;
p=[a1+b1*1i a1-b1*1i a2+b2*1i a2-b2*1i];
K = place(A, B, p);
% ----------------------------------
% �œK����
Q = diag([10 10 1 1]);
R = 10;
[K, P, e] = lqr(A, B, Q, R);
% ----------------------------------

% �Q�C���̕\��
fprintf('�Q�C��: %d %d %d %d\n', K(1), K(2), K(3), K(4));

% A - BK�s��̌ŗL�l���m�F
EigAKB = eig(A - B*K);

% ���n��ɂ����ď�Ԃ��v�Z
x = [0.0;0.1;0;0];
start = 0.0;
dt = 1e-4;
tf = 5.0;

X = zeros(tf/dt + 2, 6); % �ۑ��p�ϐ�
X(1, :) = [0, x', 0];

count = 2;
for t = start:dt:tf
    u = -K*x;
    xdot = A*x + B*u;
    x = x + xdot*dt;
    X(count, :) = [t, x', u];
    count = count + 1;
end

% �f�[�^�̍ő�l
MaxTheta = max(abs(X(:, 2)));
MaxPsi = max(abs(X(:, 3)));
MaxU = max(abs(X(:, 6)));

fprintf('�^�C���p�x�ő�l: %d\n', MaxTheta);
fprintf('�{�̌X�Ίp�ő�l: %d\n', MaxPsi);
fprintf('���͍ő�l: %d\n', MaxU);

% �f�[�^�̕\��
figure('DefaultAxesFontSize',18);
plot(X(:, 1), X(:, 2))
xlabel('Time [s]')
ylabel('Wheel Angle \theta [rad]')
grid on;

figure('DefaultAxesFontSize',18)
plot(X(:, 1), X(:, 3))
xlabel('Time [s]')
ylabel('Body Angle \psi [rad]')
grid on;
