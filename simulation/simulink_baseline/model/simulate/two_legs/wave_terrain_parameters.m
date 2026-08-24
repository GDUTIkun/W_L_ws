function terrain = wave_terrain_parameters(peakHeight, wavelength, varargin)
%WAVE_TERRAIN_PARAMETERS Piecewise-linear raised-cosine terrain geometry.
% The Simscape plant uses physical X forward, Y vertical, and Z lateral.
% peakHeight is the nonnegative crest height above the accepted flat ground.
% The profile starts and ends at zero height with zero slope:
%   h(x) = 0.5*peakHeight*(1-cos(2*pi*x/wavelength)).

parser = inputParser;
parser.addRequired("peakHeight", @(x) isnumeric(x) && isscalar(x) ...
    && isfinite(x) && x >= 0);
parser.addRequired("wavelength", @(x) isnumeric(x) && isscalar(x) ...
    && isfinite(x) && x > 0);
parser.addParameter("Enabled", peakHeight > 0, ...
    @(x) (islogical(x) || isnumeric(x)) && isscalar(x));
parser.addParameter("WavelengthCount", 2, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0 ...
        && x == round(x));
parser.addParameter("SegmentsPerWavelength", 10, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 4 ...
        && x == round(x));
parser.addParameter("Width", 2.00, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parser.addParameter("Thickness", 0.05, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parser.addParameter("LeadingEdgeX", 0.60, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.addParameter("CenterZ", 0, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.addParameter("GroundTopY", 0.025, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.parse(peakHeight, wavelength, varargin{:});

terrain = struct();
terrain.schemaVersion = "wave-terrain/1.0.0";
terrain.enabled = logical(parser.Results.Enabled);
terrain.peakHeight = double(peakHeight);
terrain.wavelength = double(wavelength);
terrain.wavelengthCount = double(parser.Results.WavelengthCount);
terrain.segmentsPerWavelength = ...
    double(parser.Results.SegmentsPerWavelength);
terrain.segmentCount = terrain.wavelengthCount ...
    * terrain.segmentsPerWavelength;
terrain.width = double(parser.Results.Width);
terrain.thickness = double(parser.Results.Thickness);
terrain.leadingEdgeX = double(parser.Results.LeadingEdgeX);
terrain.trailingEdgeX = terrain.leadingEdgeX ...
    + terrain.wavelengthCount*terrain.wavelength;
terrain.centerZ = double(parser.Results.CenterZ);
terrain.groundTopY = double(parser.Results.GroundTopY);
terrain.contactFrameMode = "horizontal-baseline";

xLocal = linspace(0, terrain.trailingEdgeX - terrain.leadingEdgeX, ...
    terrain.segmentCount + 1).';
height = 0.5*terrain.peakHeight ...
    .* (1 - cos(2*pi*xLocal/terrain.wavelength));
xWorld = terrain.leadingEdgeX + xLocal;
yWorld = terrain.groundTopY + height;

terrain.xEdges = xWorld;
terrain.heightEdges = height;
terrain.rotationAngles = zeros(terrain.segmentCount, 1);
terrain.brickDimensions = zeros(terrain.segmentCount, 3);
terrain.translations = zeros(terrain.segmentCount, 3);
for index = 1:terrain.segmentCount
    deltaX = xWorld(index + 1) - xWorld(index);
    deltaY = yWorld(index + 1) - yWorld(index);
    angle = atan2(deltaY, deltaX);
    segmentLength = hypot(deltaX, deltaY);
    topMidpoint = 0.5*[xWorld(index) + xWorld(index + 1), ...
        yWorld(index) + yWorld(index + 1)];
    terrain.rotationAngles(index) = angle;
    terrain.brickDimensions(index, :) = [segmentLength, ...
        terrain.thickness, terrain.width];
    if terrain.enabled
        terrain.translations(index, :) = [ ...
            topMidpoint(1) + 0.5*terrain.thickness*sin(angle), ...
            topMidpoint(2) - 0.5*terrain.thickness*cos(angle), ...
            terrain.centerZ];
    else
        terrain.translations(index, :) = [topMidpoint(1), -10, ...
            terrain.centerZ];
    end
end
end
