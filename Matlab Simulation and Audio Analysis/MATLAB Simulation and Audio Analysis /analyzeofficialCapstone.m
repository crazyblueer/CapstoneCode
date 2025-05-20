[audio, fs] = audioread('test_16k.wav'); 
audio = audio(:, 1); 
frame_length = 512;        
frame_shift = frame_length / 2; % 50% Overlap

num_frames = floor((length(audio) - frame_length) / frame_shift) + 1;
zcr_values = zeros(num_frames, 1);
time_axis = (0:num_frames-1) * (frame_shift / fs); % Time axis for ZCR

for i = 1:num_frames
    start_idx = (i - 1) * frame_shift + 1;
    end_idx = start_idx + frame_length - 1;
    
    frame = audio(start_idx:end_idx);
    zcr_values(i) = sum(abs(diff(sign(frame)))) / (2 * frame_length);
end

zcr_threshold = mean(zcr_values) + std(zcr_values); % Automatic threshold
vad_mask = zcr_values > zcr_threshold; % Detect frames with high ZCR (speech)

detected_speech = zeros(size(audio)); % Initialize empty array
for i = 1:num_frames
    if vad_mask(i)
        start_idx = (i - 1) * frame_shift + 1;
        end_idx = start_idx + frame_length - 1;
        detected_speech(start_idx:end_idx) = audio(start_idx:end_idx);
    end
end

figure;
subplot(3,1,1);
plot((1:length(audio)) / fs, audio);
title('Original Audio Signal');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(3,1,2);
plot(time_axis, zcr_values);
hold on;
yline(zcr_threshold, 'r--', 'Threshold');
title('Zero-Crossing Rate (ZCR) with VAD Threshold');
xlabel('Time (s)');
ylabel('ZCR');
legend('ZCR', 'VAD Threshold');
grid on;

subplot(3,1,3);
plot((1:length(audio)) / fs, audio, 'b');
hold on;
plot((1:length(audio)) / fs, detected_speech, 'r');
title('Detected Speech Segments (VAD Based on ZCR)');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Original Signal', 'Detected Speech');
