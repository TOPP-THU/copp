classdef ConstraintsRef
    %CONSTRAINTSREF Lightweight Robot constraint facade.
    %
    % This value object borrows a copp.Robot handle and forwards methods to
    % the owning Robot. It does not own native resources.

    properties (SetAccess = private)
        robot
    end

    methods
        function obj = ConstraintsRef(robot)
            arguments
                robot (1,1) copp.Robot
            end
            obj.robot = robot;
        end

        function add_velocity_limits(obj, varargin)
            obj.robot.add_velocity_limits(varargin{:});
        end

        function add_acceleration_limits(obj, varargin)
            obj.robot.add_acceleration_limits(varargin{:});
        end

        function add_jerk_limits(obj, varargin)
            obj.robot.add_jerk_limits(varargin{:});
        end

        function add_torque_limits(obj, varargin)
            obj.robot.add_torque_limits(varargin{:});
        end

        function add_raw_constraint_1st(obj, varargin)
            obj.robot.add_raw_constraint_1st(varargin{:});
        end

        function add_raw_constraint_2nd(obj, varargin)
            obj.robot.add_raw_constraint_2nd(varargin{:});
        end

        function add_raw_constraint_3rd(obj, varargin)
            obj.robot.add_raw_constraint_3rd(varargin{:});
        end

        function clear(obj, varargin)
            obj.robot.clear_constraints(varargin{:});
        end
    end
end
