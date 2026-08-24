function terrain = step_terrain_parameters(stepHeight, varargin)
%STEP_TERRAIN_PARAMETERS Build one fixed rectangular step configuration.
% The Simscape plant uses physical X forward, Y vertical, and Z lateral.
% A disabled step remains in the compiled physical network but is parked
% well below the ground so the accepted flat-ground baseline is unchanged.

parser = inputParser;
parser.addRequired("stepHeight", @(x) isnumeric(x) && isscalar(x) ...
    && isfinite(x) && x >= 0);
parser.addParameter("Enabled", stepHeight > 0, ...
    @(x) (islogical(x) || isnumeric(x)) && isscalar(x));
parser.addParameter("Length", 0.60, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parser.addParameter("Width", 2.00, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parser.addParameter("LeadingEdgeX", 0.60, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.addParameter("CenterZ", 0, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.addParameter("GroundTopY", 0.025, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.parse(stepHeight, varargin{:});

terrain = struct();
terrain.schemaVersion = "step-terrain/1.0.0";
terrain.enabled = logical(parser.Results.Enabled);
terrain.stepHeight = double(stepHeight);
terrain.stepLength = double(parser.Results.Length);
terrain.stepWidth = double(parser.Results.Width);
terrain.leadingEdgeX = double(parser.Results.LeadingEdgeX);
terrain.trailingEdgeX = terrain.leadingEdgeX + terrain.stepLength;
terrain.centerX = 0.5*(terrain.leadingEdgeX + terrain.trailingEdgeX);
terrain.centerZ = double(parser.Results.CenterZ);
terrain.groundTopY = double(parser.Results.GroundTopY);

% Brick Solid rejects a zero dimension even when the obstacle is disabled.
geometryHeight = max(terrain.stepHeight, 1e-3);
terrain.brickDimensions = [terrain.stepLength, geometryHeight, ...
    terrain.stepWidth];
if terrain.enabled
    centerY = terrain.groundTopY + 0.5*terrain.stepHeight;
else
    centerY = -10;
end
terrain.translation = [terrain.centerX, centerY, terrain.centerZ];
end
