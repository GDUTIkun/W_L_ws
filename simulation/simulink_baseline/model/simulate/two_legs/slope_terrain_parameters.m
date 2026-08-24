function terrain = slope_terrain_parameters(slopeAngle, varargin)
%SLOPE_TERRAIN_PARAMETERS Build an up-ramp/platform/down-ramp terrain.
% The Simscape plant uses physical X forward, Y vertical, and Z lateral.
% slopeAngle is expressed in radians. Disabled terrain remains connected to
% the physical network but is parked below the accepted flat-ground model.

parser = inputParser;
parser.addRequired("slopeAngle", @(x) isnumeric(x) && isscalar(x) ...
    && isfinite(x) && x >= 0 && x < pi/2);
parser.addParameter("Enabled", slopeAngle > 0, ...
    @(x) (islogical(x) || isnumeric(x)) && isscalar(x));
parser.addParameter("RampRunLength", 0.50, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parser.addParameter("PlatformLength", 0.40, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parser.addParameter("Width", 2.00, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parser.addParameter("Thickness", 0.10, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parser.addParameter("LeadingEdgeX", 0.60, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.addParameter("CenterZ", 0, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.addParameter("GroundTopY", 0.025, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.parse(slopeAngle, varargin{:});

terrain = struct();
terrain.schemaVersion = "slope-terrain/1.0.0";
terrain.enabled = logical(parser.Results.Enabled);
terrain.slopeAngle = double(slopeAngle);
terrain.downSlopeAngle = -terrain.slopeAngle;
terrain.rampRunLength = double(parser.Results.RampRunLength);
terrain.platformLength = double(parser.Results.PlatformLength);
terrain.width = double(parser.Results.Width);
terrain.thickness = double(parser.Results.Thickness);
terrain.leadingEdgeX = double(parser.Results.LeadingEdgeX);
terrain.centerZ = double(parser.Results.CenterZ);
terrain.groundTopY = double(parser.Results.GroundTopY);

terrain.rampLength = terrain.rampRunLength/cos(terrain.slopeAngle);
terrain.riseHeight = terrain.rampRunLength*tan(terrain.slopeAngle);
terrain.upEndX = terrain.leadingEdgeX + terrain.rampRunLength;
terrain.platformEndX = terrain.upEndX + terrain.platformLength;
terrain.trailingEdgeX = terrain.platformEndX + terrain.rampRunLength;

terrain.rampBrickDimensions = [terrain.rampLength, ...
    terrain.thickness, terrain.width];
terrain.platformBrickDimensions = [terrain.platformLength, ...
    terrain.thickness, terrain.width];

if terrain.enabled
    halfThicknessShift = 0.5*terrain.thickness*sin(terrain.slopeAngle);
    rampCenterY = terrain.groundTopY + 0.5*terrain.riseHeight ...
        - 0.5*terrain.thickness*cos(terrain.slopeAngle);
    terrain.upTranslation = [terrain.leadingEdgeX ...
        + 0.5*terrain.rampRunLength + halfThicknessShift, ...
        rampCenterY, terrain.centerZ];
    terrain.platformTranslation = [terrain.upEndX ...
        + 0.5*terrain.platformLength, ...
        terrain.groundTopY + terrain.riseHeight ...
        - 0.5*terrain.thickness, terrain.centerZ];
    terrain.downTranslation = [terrain.platformEndX ...
        + 0.5*terrain.rampRunLength - halfThicknessShift, ...
        rampCenterY, terrain.centerZ];
else
    terrain.upTranslation = [terrain.leadingEdgeX, -10, terrain.centerZ];
    terrain.platformTranslation = [terrain.upEndX, -10, terrain.centerZ];
    terrain.downTranslation = [terrain.platformEndX, -10, terrain.centerZ];
end
end
