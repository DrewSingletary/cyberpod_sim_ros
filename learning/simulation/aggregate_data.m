function [] = aggregate_data()
% Takes all of the data for learning and compiles into one .csv file for
% the Python learning script

% Set source directory
source_dir = './learning_data';

% Get information on number of data files in the source directory
d = dir([source_dir, '/*.mat']);
num_files = length(d);

learned_data = [];

for i = 1:num_files
    % Read in data from an episode
    filepath = sprintf('./learning_data/episode_%d_learning.mat',i);
    load(filepath);
    
    % Add the data to a matrix of data for learning
    learned_data = [learned_data; data_s];
    
end

% Write data to a CSV file for learning
csvwrite('learning_data.csv',learned_data);


end