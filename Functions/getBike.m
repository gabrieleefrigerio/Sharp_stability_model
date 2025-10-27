function [Bike, Output, P, T] = getBike(Bike)
Bike.Road.RoadSlope = deg2rad(0);
Bike.Road.Banking   = deg2rad(0);
Bike.Multibody.Roll = deg2rad(0); 
Bike.Multibody.SteeringAngle = deg2rad(0); 
data.FrontSuspension.Compression = 0;
data.RearSuspension.Compression = 0;
[Bike,P] = cinematicClosures(Bike);
[T, P, Bike] = MultibodyMatrices(Bike,P);
[Output, P] = OutputMultibody(T, P, Bike);
G = P.COG_front_assembl;
H = T.T_steering_axis;
[offset_e] = calcOffsetFromHT(H, G);
Bike.offset_e = offset_e;
end