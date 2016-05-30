function imframes = load_image_frames(image_path, filenames)

if nargin == 1
  filenames = image_path;
  image_path = 'data/test/pos/';
end

I = imread([image_path, filenames{1}]);
[M, N, p] = size(I);

imframes = zeros(M, N, p, length(filenames), 'uint8');
for i = 1:length(filenames)
  imframes(:,:,:,i) = imread([image_path, filenames{i}]);
end
