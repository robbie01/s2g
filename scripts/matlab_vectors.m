% Generate S1G reference waveforms with the MATLAB WLAN Toolbox for
% cross-checking s2g-rx (LDPC and traveling pilots have no third-party
% capture to validate against). Writes wave_<name>.cf32 (interleaved
% float32 I,Q at 2 MS/s) plus psdu_<name>.bin with the PSDU bytes.
%
%   >> matlab_vectors
%   $ s2g-rx --in wave_mcs3_ldpc_tp.cf32 --mac --quiet
%
% Requires: MATLAB R2017b+ with WLAN Toolbox (wlanS1GConfig).

cases = { ...
  'mcs0_bcc',      0, 'BCC',  false, 100; ...
  'mcs3_ldpc',     3, 'LDPC', false, 300; ...
  'mcs3_ldpc_tp',  3, 'LDPC', true,  300; ...
  'mcs7_bcc_tp',   7, 'BCC',  true,  500; ...
  'mcs8_ldpc',     8, 'LDPC', false, 1000; ...
  'mcs11_ldpc',   11, 'LDPC', false, 1200 };

for k = 1:size(cases, 1)
  name = cases{k, 1};
  cfg = wlanS1GConfig;
  cfg.ChannelBandwidth = 'CBW2';
  cfg.Preamble = 'Short';
  cfg.NumTransmitAntennas = 1;
  cfg.NumSpaceTimeStreams = 1;
  cfg.MCS = cases{k, 2};
  cfg.ChannelCoding = cases{k, 3};
  cfg.TravelingPilots = cases{k, 4};
  cfg.GuardInterval = 'Long';
  cfg.APEPLength = cases{k, 5};
  cfg.ResponseIndication = 'NDP';
  psduLen = psduLength(cfg);
  rng(k);
  psdu = randi([0 1], 8 * psduLen, 1);
  wave = wlanWaveformGenerator(psdu, cfg, 'ScramblerInitialization', 93, ...
    'IdleTime', 200e-6, 'NumPackets', 3);
  wave = [zeros(400, 1); wave; zeros(400, 1)] * 0.25;
  assert(wlanSampleRate(cfg) == 2e6);
  iq = [real(wave) imag(wave)].';
  fid = fopen(sprintf('wave_%s.cf32', name), 'w');
  fwrite(fid, single(iq(:)), 'float32');
  fclose(fid);
  bytes = uint8(sum(reshape(psdu, 8, []) .* (2 .^ (0:7)).', 1)); % bit 0 = LSB
  fid = fopen(sprintf('psdu_%s.bin', name), 'w');
  fwrite(fid, bytes, 'uint8');
  fclose(fid);
  fprintf('%s: MCS %d %s TP=%d, PSDU %d octets, %d samples\n', name, cfg.MCS, ...
    cfg.ChannelCoding, cfg.TravelingPilots, psduLen, numel(wave));
end
