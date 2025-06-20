

-- armIK
local function SolveArmIK(originCFF, targetPos, l1, l2, C1Off, Side)
    local originCF
    if Side == "Left" then
        originCF = originCFF * CFrame.new(0, 0, C1Off.X)
    elseif Side == "Right" then
        originCF = originCFF * CFrame.new(0, 0, -C1Off.X)
    end
    local localized = originCF:pointToObjectSpace(targetPos)
    local localizedUnit = localized.unit
    local l3 = localized.magnitude

    local axis = Vector3.new(0, 0, -1):Cross(localizedUnit)
    local angle = math.acos(-localizedUnit.Z)
    local planeCF = originCF * CFrame.fromAxisAngle(axis, angle)

    if l3 < math.max(l2, l1) - math.min(l2, l1) then
        return planeCF * CFrame.new(0, 0, math.max(l2, l1) - math.min(l2, l1) - l3), -math.pi / 2, math.pi
    elseif l3 > l1 + l2 then
        return planeCF, math.pi / 2, 0, l3
    else
        local a1 = -math.acos((-(l2 * l2) + (l1 * l1) + (l3 * l3)) / (2 * l1 * l3))
        local a2 = math.acos(((l2 * l2) - (l1 * l1) + (l3 * l3)) / (2 * l2 * l3))

        return planeCF, a1 + math.pi / 2, a2 - a1
    end
end

-- legIK
local function SolveLegIK(originCFF, targetPos, l1, l2, C1Off)
    local originCF = originCFF * CFrame.new(-C1Off.X, 0, 0)
    local localized = originCF:pointToObjectSpace(targetPos)
    local localizedUnit = localized.unit
    local l3 = localized.magnitude

    local axis = Vector3.new(0, 0, -1):Cross(-localizedUnit)
    local angle = math.acos(-localizedUnit.Z)
    local planeCF = originCF * CFrame.fromAxisAngle(axis, angle):Inverse()

    if l3 < math.max(l2, l1) - math.min(l2, l1) then
        return planeCF * CFrame.new(0, 0, math.max(l2, l1) - math.min(l2, l1) - l3), -math.pi / 2, math.pi
    elseif l3 > l1 + l2 then
        return planeCF, math.pi / 2, 0, l3
    else
        local a1 = -math.acos((-(l2 * l2) + (l1 * l1) + (l3 * l3)) / (2 * l1 * l3))
        local a2 = math.acos(((l2 * l2) - (l1 * l1) + (l3 * l3)) / (2 * l2 * l3))

        return planeCF, math.pi / 2 - a1, -(a2 - a1)
    end
end

-- legIKsi
local function SolveLesIK(o, t, l0, l1)
    local l = o:pointToObjectSpace(t)
    local lu = l.Unit

    local m = l.Magnitude

    local x = Vector3.new(0, 0, -1):Cross(-lu)
    local g = math.acos(-lu.Z)
    local p = o * CFrame.fromAxisAngle(x, g):Inverse()

    if m < math.max(l1, l0) - math.min(l1, l0) then
        return p * CFrame.new(0, 0, math.max(l1, l0) - math.min(l1, l0) - m), -math.pi / 2, math.pi
    elseif m > l0 + l1 then
        return p, math.pi / 2, 0
    else
        local a1 = -math.acos((-(l1 * l1) + (l0 * l0) + (m * m)) / (2 * l0 * m))
        local a2 = math.acos(((l1 * l1) - (l0 * l0) + (m * m)) / (2 * l1 * m))
        return p, math.pi / 2 - a1, -(a2 - a1)
    end
end

-- 월드 CFrame을 Motor6D의 C0(Part0 기준) Object Space CFrame으로 반환
local function WorldCFrameToC0ObjectSpace(motor6DJoint, worldCFrame)
    local part0 = motor6DJoint.Part0
    local c1Store = motor6DJoint.C1

    local relativeToPart0 = part0.CFrame:inverse() * worldCFrame * c1Store
    return relativeToPart0
end

-- R6IK 클래스 (r6)
local R6IK = {}
R6IK.__index = R6IK

-- R6IK 인스턴스
function R6IK.New(Character)
    local self = {}
    self.Torso = Character.Torso
    self.HumanoidRootPart = Character.HumanoidRootPart
    self.LeftArm = Character["Left Arm"]
    self.RightArm = Character["Right Arm"]
    self.LeftLeg = Character["Left Leg"]
    self.RightLeg = Character["Right Leg"]

    self.Motor6Ds = {
        ["Left Shoulder"] = self.Torso["Left Shoulder"],
        ["Right Shoulder"] = self.Torso["Right Shoulder"],
        ["Left Hip"] = self.Torso["Left Hip"],
        ["Right Hip"] = self.Torso["Right Hip"],
        ["RootJoint"] = self.HumanoidRootPart["RootJoint"]
    }

    self.C0s = {
        ["Left Shoulder"] = self.Motor6Ds["Left Shoulder"].C0,
        ["Right Shoulder"] = self.Motor6Ds["Right Shoulder"].C0,
        ["Left Hip"] = self.Motor6Ds["Left Hip"].C0,
        ["Right Hip"] = self.Motor6Ds["Right Hip"].C0,
        ["RootJoint"] = self.Motor6Ds["RootJoint"].C0
    }

    self.C1s = {
        ["Left Shoulder"] = self.Motor6Ds["Left Shoulder"].C1,
        ["Right Shoulder"] = self.Motor6Ds["Right Shoulder"].C1,
        ["Left Hip"] = self.Motor6Ds["Left Hip"].C1,
        ["Right Hip"] = self.Motor6Ds["Right Hip"].C1,
        ["RootJoint"] = self.Motor6Ds["RootJoint"].C1
    }

    self.Part0s = {
        ["Left Shoulder"] = self.Motor6Ds["Left Shoulder"].Part0,
        ["Right Shoulder"] = self.Motor6Ds["Right Shoulder"].Part0,
        ["Left Hip"] = self.Motor6Ds["Left Hip"].Part0,
        ["Right Hip"] = self.Motor6Ds["Right Hip"].Part0,
        ["RootJoint"] = self.Motor6Ds["RootJoint"].Part0
    }

    self.Part1s = {
        ["Left Shoulder"] = self.Motor6Ds["Left Shoulder"].Part1,
        ["Right Shoulder"] = self.Motor6Ds["Right Shoulder"].Part1,
        ["Left Hip"] = self.Motor6Ds["Left Hip"].Part1,
        ["Right Hip"] = self.Motor6Ds["Right Hip"].Part1,
        ["RootJoint"] = self.Motor6Ds["RootJoint"].Part1
    }

    self.LeftUpperArmLength = 1
    self.LeftLowerArmLength = 1
    self.RightUpperArmLength = 1
    self.RightLowerArmLength = 1
    self.LeftUpperLegLength = .8
    self.LeftLowerLegLength = 1
    self.RightUpperLegLength = .8
    self.RightLowerLegLength = 1

    self.TorsoIK = false

    return setmetatable(self, R6IK)
end

-- 팔 IK 적용 메서드
function R6IK:ArmIK(Side, Position)
    if Side == "Left" then
        local OriginCF = self.Torso.CFrame * self.C0s["Left Shoulder"]
        local C1Off = self.C1s["Left Shoulder"]
        local PlaneCF, ShoulderAngle, ElbowAngle, l3 = SolveArmIK(OriginCF, Position, self.LeftUpperArmLength, self.LeftLowerArmLength, C1Off, Side)

        local ShoulderAngleCFrame, ElbowAngleCFrame = CFrame.Angles(ShoulderAngle, 0, 0), CFrame.Angles(ElbowAngle, 0, 0)

        local ShoulderCF = PlaneCF * ShoulderAngleCFrame * CFrame.new(0, -self.LeftUpperArmLength * 0.5, 0)
        local ElbowCF = ShoulderCF * CFrame.new(0, -self.LeftUpperArmLength * 0.5, 0) * ElbowAngleCFrame * CFrame.new(0, -self.LeftLowerArmLength * 0.5, 0) * CFrame.new(0, (self.LeftArm.Size.Y - self.LeftLowerArmLength) * 0.5, 0)

        self.Motor6Ds["Left Shoulder"].C0 = WorldCFrameToC0ObjectSpace(self.Motor6Ds["Left Shoulder"], ElbowCF)
    elseif Side == "Right" then
        local OriginCF = self.Torso.CFrame * self.C0s["Right Shoulder"]
        local C1Off = self.C1s["Right Shoulder"]
        local PlaneCF, ShoulderAngle, ElbowAngle, l3 = SolveArmIK(OriginCF, Position, self.RightUpperArmLength, self.RightLowerArmLength, C1Off, Side)

        local ShoulderAngleCFrame, ElbowAngleCFrame = CFrame.Angles(ShoulderAngle, 0, 0), CFrame.Angles(ElbowAngle, 0, 0)

        local ShoulderCF = PlaneCF * ShoulderAngleCFrame * CFrame.new(0, -self.RightUpperArmLength * 0.5, 0)
        local ElbowCF = ShoulderCF * CFrame.new(0, -self.RightUpperArmLength * 0.5, 0) * ElbowAngleCFrame * CFrame.new(0, -self.RightLowerArmLength * 0.5, 0) * CFrame.new(0, (self.RightArm.Size.Y - self.RightLowerArmLength) * 0.5, 0)

        self.Motor6Ds["Right Shoulder"].C0 = WorldCFrameToC0ObjectSpace(self.Motor6Ds["Right Shoulder"], ElbowCF)
    end
end

-- 다리 IK 적용 메서드
function R6IK:LegIK(Side, Position)
    if Side == "Left" then
        local OriginCF = self.Torso.CFrame * self.C0s["Left Hip"]
        local C1Off = self.C1s["Left Hip"]
        local PlaneCF, ShoulderAngle, ElbowAngle, l3 = SolveLegIK(OriginCF * CFrame.Angles(0, math.pi / 2, 0), Position, self.LeftUpperLegLength, self.LeftLowerLegLength, C1Off)

        local HipAngleCFrame, KneeAngleCFrame = CFrame.Angles(ShoulderAngle, 0, 0), CFrame.Angles(ElbowAngle, 0, 0)

        local HipCF = PlaneCF * HipAngleCFrame * CFrame.new(0, -self.LeftUpperLegLength * 0.5, 0)
        local KneeCF = HipCF * CFrame.new(0, -self.LeftUpperLegLength * 0.5, 0) * KneeAngleCFrame * CFrame.new(-0.1, -self.LeftLowerLegLength * 0.5, 0) * CFrame.new(0, (self.LeftLeg.Size.Y - self.LeftLowerLegLength) * 0.5, 0) * CFrame.Angles(0, 0.15, 0)

        return WorldCFrameToC0ObjectSpace(self.Motor6Ds["Left Hip"], KneeCF)
    elseif Side == "Right" then
        local OriginCF = self.Torso.CFrame * self.C0s["Right Hip"]
        local C1Off = self.C1s["Right Hip"]
        local PlaneCF, ShoulderAngle, ElbowAngle, l3 = SolveLegIK(OriginCF * CFrame.Angles(0, -math.pi / 2, 0), Position, self.RightUpperLegLength, self.RightLowerLegLength, C1Off)

        local HipAngleCFrame, KneeAngleCFrame = CFrame.Angles(ShoulderAngle, 0, 0), CFrame.Angles(ElbowAngle, 0, 0)

        local HipCF = PlaneCF * HipAngleCFrame * CFrame.new(0, -self.RightUpperLegLength * 0.5, 0)
        local KneeCF = HipCF * CFrame.new(0, -self.RightUpperLegLength * 0.5, 0) * KneeAngleCFrame * CFrame.new(0.1, -self.RightLowerLegLength * 0.5, 0) * CFrame.new(0, (self.RightLeg.Size.Y - self.RightLowerLegLength) * 0.5, 0) * CFrame.Angles(0, -0.15, 0)

        return WorldCFrameToC0ObjectSpace(self.Motor6Ds["Right Hip"], KneeCF)
    end
end

return R6IK
