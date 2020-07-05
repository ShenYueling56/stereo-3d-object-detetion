%first run stereoCameraCalibrator
RD1 = stereoParams.CameraParameters1.RadialDistortion;
TD1 = stereoParams.CameraParameters1.TangentialDistortion;
D1 = [RD1(1), RD1(2), TD1(1), TD1(2), RD1(3)]
K1 = stereoParams.CameraParameters1.IntrinsicMatrix'

RD2 = stereoParams.CameraParameters2.RadialDistortion;
TD2 = stereoParams.CameraParameters2.TangentialDistortion;
D2 = [RD2(1), RD2(2), TD2(1), TD2(2), RD2(3)]
K2 = stereoParams.CameraParameters2.IntrinsicMatrix'

size = stereoParams.CameraParameters1.ImageSize

rot = stereoParams.RotationOfCamera2
trans = stereoParams.TranslationOfCamera2

% T=eye(4);
% T(1:3,1:3)=rot;
% T(1:3,4)=trans;
% T=inv(T);
% rot=T(1:3,1:3)
% trans=T(1:3,4)


E = stereoParams.EssentialMatrix
F = stereoParams.FundamentalMatrix
% copy the parameters to stereocalibrateresult_matlab.txt
