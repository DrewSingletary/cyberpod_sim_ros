d_drift_in = 8;
d_act_in = 8;
d_drift_hidden = 30;
d_drift_hidden_2 = 30;
d_act_hidden = 30;
d_act_hidden_2 = 30;
d_drift_out = 1;
d_act_out = 1;

delete weights.h
fidW = fopen( 'weights.h', 'wt' );

fprintf(fidW,'const uint32_t d_drift_in = %i;\n',d_drift_in);
fprintf(fidW,'const uint32_t d_act_in = %i;\n',d_act_in);
fprintf(fidW,'const uint32_t d_drift_hidden = %i;\n',d_drift_hidden);
fprintf(fidW,'const uint32_t d_act_hidden = %i;\n',d_act_hidden);
fprintf(fidW,'const uint32_t d_drift_hidden_2 = %i;\n',d_drift_hidden_2);
fprintf(fidW,'const uint32_t d_act_hidden_2 = %i;\n',d_act_hidden_2);
fprintf(fidW,'const uint32_t d_drift_out = %i;\n',d_drift_out);
fprintf(fidW,'const uint32_t d_act_out = %i;\n',d_act_out);

fprintf(fidW,'\n');

drift_w_1 = csvread('drift_w_1.csv');
fprintf(fidW,'const double w_1_drift[d_drift_hidden*d_drift_in] = {');

for j = 1:d_drift_in
    for i = 1:d_drift_hidden
        if j == d_drift_in && i == d_drift_hidden
            fprintf(fidW,'%d};',drift_w_1(i,j));
        else
            fprintf(fidW,'%d,',drift_w_1(i,j));
        end
    end
end

fprintf(fidW,'\n');


drift_w_2 = csvread('drift_w_2.csv');
fprintf(fidW,'const double w_2_drift[d_drift_hidden*d_drift_hidden_2] = {');
% drift_w_2 = drift_w_2';

for j = 1:d_drift_hidden
    for i = 1:d_drift_hidden_2
        if i == d_drift_hidden_2 && j == d_drift_hidden
            fprintf(fidW,'%d};',drift_w_2(i,j));
        else
            fprintf(fidW,'%d,',drift_w_2(i,j));
        end
    end
end

fprintf(fidW,'\n');

drift_w_3 = csvread('drift_w_3.csv');
fprintf(fidW,'const double w_3_drift[d_drift_hidden_2*d_drift_out] = {');
% drift_w_3 = drift_w_3';

for j = 1:d_drift_hidden_2
    for i = 1:d_drift_out
        if i == d_drift_out && j == d_drift_hidden_2
            fprintf(fidW,'%d};',drift_w_3(i,j));
        else
            fprintf(fidW,'%d,',drift_w_3(i,j));
        end
    end
end

fprintf(fidW,'\n');

drift_b_1 = csvread('drift_b_1.csv');
fprintf(fidW,'const double b_1_drift[d_drift_hidden] = {');

for i = 1:d_drift_hidden
    if i == d_drift_hidden
        fprintf(fidW,'%d};',drift_b_1(i));
    else
        fprintf(fidW,'%d,',drift_b_1(i));
    end
end

fprintf(fidW,'\n');

drift_b_2 = csvread('drift_b_2.csv');
fprintf(fidW,'const double b_2_drift[d_drift_hidden_2] = {');

for i = 1:d_drift_hidden_2
    if i == d_drift_hidden_2
        fprintf(fidW,'%d};',drift_b_2(i));
    else
        fprintf(fidW,'%d,',drift_b_2(i));
    end
end

fprintf(fidW,'\n');

drift_b_3 = csvread('drift_b_3.csv');
fprintf(fidW,'const double b_3_drift[d_drift_out] = {');

for i = 1:d_drift_out
    if i == d_drift_out
        fprintf(fidW,'%d};',drift_b_3(i));
    else
        fprintf(fidW,'%d,',drift_b_3(i));
    end
end

% act now

fprintf(fidW,'\n');

act_w_1 = csvread('act_w_1.csv');
fprintf(fidW,'const double w_1_act[d_drift_hidden*d_act_in] = {');

for j = 1:d_act_in
    for i = 1:d_act_hidden
        if j == d_act_in && i == d_act_hidden
            fprintf(fidW,'%d};',act_w_1(i,j));
        else
            fprintf(fidW,'%d,',act_w_1(i,j));
        end
    end
end

fprintf(fidW,'\n');

act_w_2 = csvread('act_w_2.csv');
fprintf(fidW,'const double w_2_act[d_act_hidden*d_act_hidden_2] = {');
% act_w_2 = act_w_2';

for j = 1:d_act_hidden
    for i = 1:d_act_hidden_2
        if i == d_act_hidden_2 && j == d_act_hidden
            fprintf(fidW,'%d};',act_w_2(i,j));
        else
            fprintf(fidW,'%d,',act_w_2(i,j));
        end
    end
end

fprintf(fidW,'\n');

act_w_3 = csvread('act_w_3.csv');
fprintf(fidW,'const double w_3_act[d_act_hidden_2*d_act_out] = {');
% act_w_3 = act_w_3';

for j = 1:d_act_hidden_2
    for i = 1:d_act_out
        if i == d_act_out && j == d_act_hidden_2
            fprintf(fidW,'%d};',act_w_3(i,j));
        else
            fprintf(fidW,'%d,',act_w_3(i,j));
        end
    end
end

fprintf(fidW,'\n');

act_b_1 = csvread('act_b_1.csv');
fprintf(fidW,'const double b_1_act[d_act_hidden] = {');

for i = 1:d_act_hidden
    if i == d_act_hidden
        fprintf(fidW,'%d};',act_b_1(i));
    else
        fprintf(fidW,'%d,',act_b_1(i));
    end
end

fprintf(fidW,'\n');

act_b_2 = csvread('act_b_2.csv');
fprintf(fidW,'const double b_2_act[d_act_hidden_2] = {');

for i = 1:d_act_hidden_2
    if i == d_act_hidden_2
        fprintf(fidW,'%d};',act_b_2(i));
    else
        fprintf(fidW,'%d,',act_b_2(i));
    end
end

fprintf(fidW,'\n');

act_b_3 = csvread('act_b_3.csv');
fprintf(fidW,'const double b_3_act[d_act_out] = {');

for i = 1:d_act_out
    if i == d_act_out
        fprintf(fidW,'%d};',act_b_3(i));
    else
        fprintf(fidW,'%d,',act_b_3(i));
    end
end

fclose(fidW);