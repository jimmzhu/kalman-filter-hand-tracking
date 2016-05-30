function [lhd, rhd, lhp, rhp] = format_data_labels(file_dir, file_prefix)

label_filenames = dir([file_dir '/' file_prefix '*.txt']);
total_frames = length(label_filenames);

lhd = -1 * ones(total_frames, 4);
rhd = -1 * ones(total_frames, 4);
lhp = -1 * ones(total_frames, 4);
rhp = -1 * ones(total_frames, 4);

for i = 1:total_frames
  label_filename = label_filenames(i).name;
  label = importdata([file_dir '/' label_filename], ' ', 1);
  total_labels = length(label.textdata) - 1;

  for j = 1:total_labels
    if strcmp(label.textdata{j+1}, 'leftHand_driver')
      lhd(i,:) = label.data(j,1:4);
    elseif strcmp(label.textdata{j+1}, 'rightHand_driver')
      rhd(i,:) = label.data(j,1:4);
    elseif strcmp(label.textdata{j+1}, 'leftHand_passenger')
      lhp(i,:) = label.data(j,1:4);
    elseif strcmp(label.textdata{j+1}, 'rightHand_passenger')
      rhp(i,:) = label.data(j,1:4);
    end
  end
end
