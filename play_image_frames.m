function play_image_frames(mat_filename, subdir)

if nargin < 2
  subdir = 'test';
end
image_path = sprintf('data/%s/pos/', subdir);

load(mat_filename);
imframes = load_image_frames(image_path, frames);

for i = 1:length(frames)
  imshow(imframes(:,:,:,i));
  rectangle('Position', lhd(i,:), 'EdgeColor', 'red', 'LineWidth', 2, 'Curvature', 0.1);
  rectangle('Position', rhd(i,:), 'EdgeColor', 'green', 'LineWidth', 2, 'Curvature', 0.1);
  rectangle('Position', lhp(i,:), 'EdgeColor', 'magenta', 'LineWidth', 2, 'Curvature', 0.1);
  rectangle('Position', rhp(i,:), 'EdgeColor', 'cyan', 'LineWidth', 2, 'Curvature', 0.1);
  refresh; pause(0.05);
end
