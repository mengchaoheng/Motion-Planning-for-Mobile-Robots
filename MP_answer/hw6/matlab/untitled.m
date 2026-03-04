clc;clear;
close all
v_max = 400;
a_max = 400;
color = ['r', 'b', 'm', 'g', 'k', 'c', 'c'];
marker=['+', '<', '*', 'x', '.', 'v', 'square'];
%% specify the center points of the flight corridor and the region of corridor
path = [50, 50;
       100, 120;
       180, 150;
       250, 80;
       280, 0];
% path = [00, 0;
%        980, 0;
%        1000,0]; 
% path = [00, 0;
%        500, 100;
%        1000,200;
%        1500,300]; 
% path = [0, 0;
%        1000, 0]; 
% x_length = 100;
x_length = 100;
y_length = 100;

n_order = 5;   % 8 control points
n_seg = size(path, 1); % Each corridor has a curve

corridor = zeros(4, n_seg);
for i = 1:n_seg
    corridor(:, i) = [path(i, 1), path(i, 2), x_length/2, y_length/2]';
end
% corridor(3, 2) = 1500/2;corridor(4, 2) = 300/2;
%% specify ts for each segment
ts = zeros(n_seg, 1);
for i = 1:n_seg
    % if i~=1 && i~=n_seg
    %   % ts(i,1) = 1*sqrt((corridor(3, i)*2)^2+(corridor(4, i)*2)^2)/v_max;
    %   % ts(i,1) = (norm(path(3, :)-path(1, :))/v_max)*1;
    %   t1=v_max/a_max;p1=a_max*t1^2/2;
    %   ts(i,1) =((norm(path(3, :)-path(1, :)-2*p1))/v_max)*1;
    % else
    %     % led=sqrt((corridor(3, i)*2)^2+(corridor(4, i)*2)^2)/2;
    %     led=norm(path(2, :)-path(1, :));
    %     t1=v_max/a_max;p1=a_max*t1^2/2;
    %     if led<=p1
    %         ts(i,1)=sqrt(2*led/a_max);
    %     else
    %         t2=(led-p1)/v_max;
    %         ts(i,1)=(t1+0)*1;
    %     end
    % 
    % end
    ts(i,1) = 1;
end
% ts=[2;50;2]*1;
ts=[1;1;1;1;1]*1;
% ts=ts*1
poly_coef_x = MinimumSnapCorridorBezierSolver(1, path(:, 1), corridor, ts, n_seg, n_order, v_max, a_max);
poly_coef_y = MinimumSnapCorridorBezierSolver(2, path(:, 2), corridor, ts, n_seg, n_order, v_max, a_max);

%% display the trajectory and cooridor
figure('Name', 'corridor', 'Position', [1200, 100, 800, 800]);
% plot(path(:,1), path(:,2), '*r'); hold on;
% for i = 1:n_seg
%     plot_rect([corridor(1,i);corridor(2,i)], corridor(3, i), corridor(4,i));hold on;
% end
hold on;
%% 计算并绘制X、Y方向的位置、速度、加速度随时间变化曲线

% 初始化存储数组
time_array = []; % 时间轴
x_positions = []; % X方向位置
y_positions = []; % Y方向位置
x_velocities = []; % X方向速度
y_velocities = []; % Y方向速度
x_accelerations = []; % X方向加速度
y_accelerations = []; % Y方向加速度
x_jerk=[];
y_jerk=[];

total_time_prev = 0; % 初始化累计时间

% 遍历每个分段
for k = 1:n_seg
    % 提取当前分段的控制点 (X和Y坐标)
    control_points_x = poly_coef_x((k-1)*(n_order+1)+1 : k*(n_order+1));
    control_points_y = poly_coef_y((k-1)*(n_order+1)+1 : k*(n_order+1));
    
    % 遍历当前分段内的每个参数t
    for t = 0:0.005:1
        % 计算当前点的位置 (你原有的代码)
        x_pos_current = 0.0;
        y_pos_current = 0.0;
        for i = 0:n_order
            C=nchoosek(n_order, i);
            basis_p =  t^i * (1-t)^(n_order-i);
            x_pos_current = x_pos_current + ts(k) * C * poly_coef_x((k-1)*(n_order+1)+i+1) * basis_p;
            y_pos_current = y_pos_current + ts(k) * C * poly_coef_y((k-1)*(n_order+1)+i+1) * basis_p;
        end
        
        % 计算当前点的速度 (一阶导数)
        vx_current = 0.0;
        vy_current = 0.0;
        for i = 0:n_order-1
            Cv=nchoosek(n_order-1, i);
            basis_p_v = t^i * (1-t)^(n_order-1-i);
            vx_current = vx_current + Cv * n_order * (control_points_x(i+2) - control_points_x(i+1)) * basis_p_v;
            vy_current = vy_current + Cv * n_order * (control_points_y(i+2) - control_points_y(i+1)) * basis_p_v;
        end
        
        % 计算当前点的加速度 (二阶导数)
        ax_current = 0.0;
        ay_current = 0.0;
        if n_order >= 2
            for i = 0:n_order-2
                Ca=nchoosek(n_order-2, i);
                basis_p_a =  t^i * (1-t)^(n_order-2-i);
                delta2_x = control_points_x(i+3) - 2*control_points_x(i+2) + control_points_x(i+1);
                delta2_y = control_points_y(i+3) - 2*control_points_y(i+2) + control_points_y(i+1);
                ax_current = ax_current + Ca * n_order*(n_order-1) * delta2_x * basis_p_a / (ts(k));
                ay_current = ay_current + Ca * n_order*(n_order-1) * delta2_y * basis_p_a / (ts(k));
            end
        end 

        % 计算当前点的jerk (三阶导数)
        jx_current = 0.0;
        jy_current = 0.0;
        if n_order >= 3
            for i = 0:n_order-3
                Cj=nchoosek(n_order-3, i);
                basis_p_j =  t^i * (1-t)^(n_order-3-i);
                delta3_x = control_points_x(i+4) - 3*control_points_x(i+3) + 3*control_points_x(i+2) -control_points_x(i+1);
                delta3_y = control_points_y(i+4) - 3*control_points_y(i+3) + 3*control_points_y(i+2) -control_points_y(i+1);
                jx_current = jx_current + Cj * n_order*(n_order-1)*(n_order-2) * delta3_x * basis_p_j / (ts(k))^2;
                jy_current = jy_current + Cj * n_order*(n_order-1)*(n_order-2) * delta3_y * basis_p_j / (ts(k))^2;
            end
        end 


        
        % 计算当前真实时间
        current_real_time = total_time_prev + ts(k) * t;
        
        % 存储数据
        time_array(end+1) = current_real_time;
        x_positions(end+1) = x_pos_current;
        y_positions(end+1) = y_pos_current;
        x_velocities(end+1) = vx_current;
        y_velocities(end+1) = vy_current;
        x_accelerations(end+1) = ax_current;
        y_accelerations(end+1) = ay_current;
        x_jerk(end+1) = jx_current;
        y_jerk(end+1) = jy_current;
    end
    % if(1)
    % f = plot(x_positions((k-1)*200+1:k*200),y_positions((k-1)*200+1:k*200));
    % f.Color = color(k);
    % scatter(ts(k) * poly_coef_x((k-1)*(n_order+1)+1:k*(n_order+1)),...
    % ts(k) * poly_coef_y((k-1)*(n_order+1)+1:k*(n_order+1)),color(k));
    % end
    
    f2=plot(x_velocities, x_accelerations); 
    f2.Marker=marker(k);
    % 更新累计时间
    total_time_prev = total_time_prev + ts(k);
end

%% 绘制X方向的位置、速度、加速度随时间变化图
figure('Name', 'X方向运动状态', 'Position', [0, 100, 600, 800]);

subplot(5,1,1);
plot(time_array, x_positions, 'b-', 'LineWidth', 1.5);hold on;
% plot([ts(1) ts(1)],[-1000 1000],'r-.', 'LineWidth', 1.5');hold on;
% plot([ts(1)+ts(2) ts(1)+ts(2)],[-1000 1000],'g-', 'LineWidth', 1.5');
xlabel('时间 (s)');
ylabel('X位置');
title('X方向位置 vs 时间');
grid on;

subplot(5,1,2);
plot(time_array, x_velocities, 'r-', 'LineWidth', 1.5);hold on;
% plot([ts(1) ts(1)],[-100 100],'r-.', 'LineWidth', 1.5');hold on;
% plot([ts(1)+ts(2) ts(1)+ts(2)],[-100 100],'g-', 'LineWidth', 1.5');
xlabel('时间 (s)');
ylabel('X速度');
title('X方向速度 vs 时间');
grid on;

subplot(5,1,3);
plot(time_array, x_accelerations, 'g-', 'LineWidth', 1.5);hold on;
% plot([ts(1) ts(1)],[-100 100],'r-.', 'LineWidth', 1.5');hold on;
% plot([ts(1)+ts(2) ts(1)+ts(2)],[-100 100],'g-', 'LineWidth', 1.5');
xlabel('时间 (s)');
ylabel('X加速度');
title('X方向加速度 vs 时间');
grid on;
subplot(5,1,4);
plot(time_array, x_jerk, 'g-', 'LineWidth', 1.5);hold on;
subplot(5,1,5);

plot(x_velocities, x_accelerations, 'g-', 'LineWidth', 1.5);hold on;
%% 绘制Y方向的位置、速度、加速度随时间变化图
figure('Name', 'Y方向运动状态', 'Position', [600, 100, 600, 800]);

subplot(3,1,1);
plot(time_array, y_positions, 'b-', 'LineWidth', 1.5);hold on;
% plot([ts(1) ts(1)],[-1000 1000],'r-.', 'LineWidth', 1.5');hold on;
% plot([ts(1)+ts(2) ts(1)+ts(2)],[-1000 1000],'g-', 'LineWidth', 1.5');
xlabel('时间 (s)');
ylabel('Y位置');
title('Y方向位置 vs 时间');
grid on;

subplot(3,1,2);
plot(time_array, y_velocities, 'r-', 'LineWidth', 1.5);hold on;
% plot([ts(1) ts(1)],[-100 100],'r-.', 'LineWidth', 1.5');hold on;
% plot([ts(1)+ts(2) ts(1)+ts(2)],[-100 100],'g-', 'LineWidth', 1.5');
xlabel('时间 (s)');
ylabel('Y速度');
title('Y方向速度 vs 时间');
grid on;

subplot(3,1,3);
plot(time_array, y_accelerations, 'g-', 'LineWidth', 1.5);hold on;
% plot([ts(1) ts(1)],[-100 100],'r-.', 'LineWidth', 1.5');hold on;
% plot([ts(1)+ts(2) ts(1)+ts(2)],[-100 100],'g-', 'LineWidth', 1.5');
xlabel('时间 (s)');
ylabel('Y加速度');
title('Y方向加速度 vs 时间');
grid on;



% %% 绘制速度与加速度矢量场（可选）
% figure('Name', '速度与加速度矢量场', 'Position', [100, 100, 1200, 500]);
% 
% % 速度矢量场
% subplot(1,2,1);
% skip = 20; % 每隔20个点画一个矢量，避免过于密集
% quiver(x_positions(1:skip:end), y_positions(1:skip:end), x_velocities(1:skip:end), y_velocities(1:skip:end), 0.5, 'b');
% hold on;
% plot(x_positions, y_positions, 'r-', 'LineWidth', 2); % 重叠画出原始轨迹
% axis equal;
% grid on;
% xlabel('X位置');
% ylabel('Y位置');
% title('速度矢量场');
% hold off;
% 
% % 加速度矢量场
% subplot(1,2,2);
% quiver(x_positions(1:skip:end), y_positions(1:skip:end), x_accelerations(1:skip:end), y_accelerations(1:skip:end), 0.5, 'g');
% hold on;
% plot(x_positions, y_positions, 'r-', 'LineWidth', 2); % 重叠画出原始轨迹
% axis equal;
% grid on;
% xlabel('X位置');
% ylabel('Y位置');
% title('加速度矢量场');
% hold off;

function poly_coef = MinimumSnapCorridorBezierSolver(axis, waypoints, corridor, ts, n_seg, n_order, v_max, a_max)
    d_order=3;
    start_cond = [waypoints(1), zeros(1,d_order-1)];
    end_cond   = [waypoints(end), zeros(1,d_order-1)];  
    
    
    %% #####################################################
    % STEP 1: compute Q_0 of c'Q_0c
    [Q, M]  = getQM(n_seg, n_order, d_order, ts);
    Q_0 = M'*Q*M;
    Q_0 = nearestSPD(Q_0);
    
    %% #####################################################
    % STEP 2: get Aeq and beq
    [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond);
    
    %% #####################################################
    % STEP 3: get corridor_range, Aieq and bieq 
    
    % STEP 3.1: get corridor_range of x-axis or y-axis,
    % you can define corridor_range as [p1_min, p1_max;
    %                                   p2_min, p2_max;
    %                                   ...,
    %                                   pn_min, pn_max ];
    corridor_range = zeros(n_seg,2);
    for k = 1:n_seg
        corridor_range(k,:) = [corridor(axis,k) - corridor(axis+2,k), corridor(axis,k) + corridor(axis+2,k)];
    end

    
    % STEP 3.2: get Aieq and bieq
    [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max);
    
    f = zeros(size(Q_0,1),1);
    poly_coef = quadprog(Q_0,f,Aieq, bieq, Aeq, beq);
end

function plot_rect(center, x_r, y_r)
    p1 = center+[-x_r;-y_r];
    p2 = center+[-x_r;y_r];
    p3 = center+[x_r;y_r];
    p4 = center+[x_r;-y_r];
    plot_line(p1,p2);
    plot_line(p2,p3);
    plot_line(p3,p4);
    plot_line(p4,p1);
end

function plot_line(p1,p2)
    a = [p1(:),p2(:)];    
    plot(a(1,:),a(2,:),'b');
end