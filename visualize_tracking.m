raw_results_mat = 'TestSetResults.mat';
formatted_results_mat = 'test_v1_tracking.mat';
labels_mat = 'test_v1_labels.mat';
tracking_visual_mat = 'test_v1_visual.mat';
image_dir = 'data/test/pos/';
wheel_region = [30 270 276 76];

output_dir = 'test_v1/tracking_output/';
load(labels_mat);

if ~exist('imframes', 'var')
  if ~exist(formatted_results_mat, 'file') && ~exist('pxl', 'var')
    lhd_center = [pxld pyld];
    rhd_center = [pxrd pyrd];
    lhp_center = [pxlp pylp];
    rhp_center = [pxrp pyrp];
    lhd_center_t = stateSavesl(:,1:2);
    rhd_center_t = stateSavesr(:,1:2);
    lhp_center_t = stateSaveslp(:,1:2);
    rhp_center_t = stateSavesrp(:,1:2);
    save(formatted_results_mat, '-mat7-binary', ...
         'lhd_center', 'rhd_center', 'lhd_center_t', 'rhd_center_t', ...
         'lhp_center', 'rhp_center', 'lhp_center_t', 'rhp_center_t');
  elseif ~exist(formatted_results_mat, 'file')
    load(raw_results_mat);
    lhd_center = [pxl pyl];
    rhd_center = [pxr pyr];
    lhd_center_t = stateSavesl(:,1:2);
    rhd_center_t = stateSavesr(:,1:2);
    save(formatted_results_mat, '-mat7-binary', ...
         'lhd_center', 'rhd_center', 'lhd_center_t', 'rhd_center_t');
  else
    load(formatted_results_mat);
  end

  % set to true when you have the correct hand center position data
  check = true;
  if (check)
    lhd_center_gt = [lhd(:,1) + 0.5*lhd(:,3), lhd(:,2) + 0.5*lhd(:,4)];
    rhd_center_gt = [rhd(:,1) + 0.5*rhd(:,3), rhd(:,2) + 0.5*rhd(:,4)];
    lhp_center_gt = [lhp(:,1) + 0.5*lhp(:,3), lhp(:,2) + 0.5*lhp(:,4)];
    rhp_center_gt = [rhp(:,1) + 0.5*rhp(:,3), rhp(:,2) + 0.5*rhp(:,4)];
    center_gt = cat(3, lhd_center_gt, rhd_center_gt, ...
                       lhp_center_gt, rhp_center_gt);
    lrhdp = cat(3, lhd, rhd, lhp, rhp);
%    assert(isequal(lhd_center, lhd_center_gt));
%    assert(isequal(rhd_center, rhd_center_gt));
  else
    [N, ~] = size(lhd);
    lhd_center = lhd_center(1:N,:);
    rhd_center = rhd_center(1:N,:);
    lhd_center_t = lhd_center_t(1:N,:);
    rhd_center_t = rhd_center_t(1:N,:);
  end

  imframes = load_image_frames(image_dir, frames);
  [M, N, ~, F] = size(imframes);
end

wheel_topLeft = wheel_region(1:2);
wheel_bottomRight = wheel_region(1:2)+wheel_region(3:4);
lrhdp_center = cat(3, lhd_center, rhd_center, lhp_center, rhp_center);
hands_on_wheel = sum(prod(lrhdp_center > wheel_topLeft, 2) .* ...
                     prod(lrhdp_center < wheel_bottomRight, 2), 3);
text_position = [N-60,20];

lhdi = lrhdp(1,:,find(prod(lhd_center(1,:) == center_gt(1,:,:), 2), 1));
rhdi = lrhdp(1,:,find(prod(rhd_center(1,:) == center_gt(1,:,:), 2), 1));
lhpi = lrhdp(1,:,find(prod(lhp_center(1,:) == center_gt(1,:,:), 2), 1));
rhpi = lrhdp(1,:,find(prod(rhp_center(1,:) == center_gt(1,:,:), 2), 1));

fig = figure(1);
frames = cell(1, F);
for i = 2:F
  frames{i} = sprintf('%05d.png', i);
  imshow(imframes(:,:,:,i)); hold on;

  rectangle('Position', wheel_region, 'EdgeColor', 'k', 'LineWidth', 2, 'Curvature', 0.1);
  str = sprintf('%d', hands_on_wheel(i));
  text(N-60,40, str, 'FontSize', 32, 'FontWeight', 'bold', ...
                     'EdgeColor', 'none');

  lhdi_t = [lhd_center_t(i,:)-0.5*lhdi(3:4), lhdi(3:4)];
  rhdi_t = [rhd_center_t(i,:)-0.5*rhdi(3:4), rhdi(3:4)];
  lhpi_t = [lhp_center_t(i,:)-0.5*lhpi(3:4), lhpi(3:4)];
  rhpi_t = [rhp_center_t(i,:)-0.5*rhpi(3:4), rhpi(3:4)];
  lhdi = lrhdp(i,:,find(prod(lhd_center(i,:) == center_gt(i,:,:), 2), 1));
  rhdi = lrhdp(i,:,find(prod(rhd_center(i,:) == center_gt(i,:,:), 2), 1));
  lhpi = lrhdp(i,:,find(prod(lhp_center(i,:) == center_gt(i,:,:), 2), 1));
  rhpi = lrhdp(i,:,find(prod(rhp_center(i,:) == center_gt(i,:,:), 2), 1));
  rectangle('Position', lhdi, 'EdgeColor', 'b', 'LineWidth', 2, 'Curvature', 0.1);
  rectangle('Position', rhdi, 'EdgeColor', 'r', 'LineWidth', 2, 'Curvature', 0.1);
  rectangle('Position', lhpi, 'EdgeColor', 'r', 'LineWidth', 2, 'Curvature', 0.1);
  rectangle('Position', rhpi, 'EdgeColor', 'b', 'LineWidth', 2, 'Curvature', 0.1);
  rectangle('Position', lhdi_t, 'EdgeColor', 'c', 'LineWidth', 1, 'Curvature', 0.1);
  rectangle('Position', rhdi_t, 'EdgeColor', 'm', 'LineWidth', 1, 'Curvature', 0.1);
  rectangle('Position', lhpi_t, 'EdgeColor', 'm', 'LineWidth', 1, 'Curvature', 0.1);
  rectangle('Position', rhpi_t, 'EdgeColor', 'c', 'LineWidth', 1, 'Curvature', 0.1);

  plot(lhd_center(i,1), lhd_center(i,2), 'b-o', 'LineWidth', 2, ...
       rhd_center(i,1), rhd_center(i,2), 'r-o', 'LineWidth', 2, ...
       lhp_center(i,1), lhp_center(i,2), 'r-o', 'LineWidth', 2, ...
       rhp_center(i,1), rhp_center(i,2), 'b-o', 'LineWidth', 2, ...
       lhd_center_t(i,1), lhd_center_t(i,2), 'c-x', 'LineWidth', 2, ...
       rhd_center_t(i,1), rhd_center_t(i,2), 'm-x', 'LineWidth', 2, ...
       lhp_center_t(i,1), lhp_center_t(i,2), 'm-x', 'LineWidth', 2, ...
       rhp_center_t(i,1), rhp_center_t(i,2), 'c-x', 'LineWidth', 2);

  set(gca, 'position', [0 0 1 1], 'units', 'normalized');
  saveas(fig, [output_dir frames{i}]); hold off;
end

save(tracking_visual_mat, '-mat7-binary', 'output_dir', 'frames');
