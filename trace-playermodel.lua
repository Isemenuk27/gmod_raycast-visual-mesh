if ( SERVER ) then return end

-- Almost all code used from here:
-- https://gist.github.com/ZakBlystone/c3d337bacdbfe6a71ec3c32f6506c13b

local ENABLE_DEMONSTRATION = true

local tab = FindMetaTable("Vector")
local set = tab.Set
local add = tab.Add
local sub = tab.Sub
local dot = tab.Dot
local mul = tab.Mul
local lensqr = tab.LengthSqr
local distsqr = tab.DistToSqr
local norm = tab.Normalize
local unpackvec = tab.Unpack
local setunpacked = tab.SetUnpacked

local TriBarycentric
local RayIntersectPlane
local TriNormal
local RayIntersectTriangle
local RayIntersectTriangles
local TransformNormal

TransformNormal = function( mtx, normal )
	local A00, A01, A02, A03, A10, A11, A12, A13, A20, A21, A22, A23, A30, A31, A32, A33 = mtx:Unpack()
	local x, y, z = unpackvec( normal )
	return Vector( A00 * x + A01 * y + A02 * z, A10 * x + A11 * y + A12 * z, A20 * x + A21 * y + A22 * z )
end

local __vedge1 = Vector()
local __vedge2 = Vector()
local __vedge3 = Vector()

TriBarycentric = function(a,b,c,point)

    set( __vedge1, b )
    sub( __vedge1, a )
    set( __vedge2, c )
    sub( __vedge2, a )
    set( __vedge3, point )
    sub( __vedge3, a )
	local v0 = __vedge1
	local v1 = __vedge2
	local v2 = __vedge3
	local d00 = dot( v0, v0 )
	local d01 = dot( v0, v1 )
	local d11 = dot( v1, v1 )
	local d20 = dot( v2, v0 )
	local d21 = dot( v2, v1 )
	local denom = d00 * d11 - d01 * d01

	if ( denom == 0 ) then 
		return 0, 0, 0 
	end

    local idenom = 1 / denom
	local v = ( d11 * d20 - d01 * d21 ) * idenom
	local w = ( d00 * d21 - d01 * d20 ) * idenom
	local u = 1 - v - w

	return u, v, w
end

local __vraytemp = Vector()

RayIntersectPlane = function( normal, origin, p, dir )
	local d = dot( normal, dir )

	if ( d < -.0001 ) then
        set( __vraytemp, origin )
        sub( __vraytemp, p )
		t = dot( __vraytemp, normal ) / d
		return t >= 0, t
	end

	return false
end

local __vtemp, __vtemp2, __vtemp3 = Vector(),  Vector(),  Vector()

TriNormal = function(a,b,c)
	set( __vtemp, a )
	sub( __vtemp, b )
	set( __vtemp2, c )
	sub( __vtemp2, a )

    local a1, a2, a3 = unpackvec( __vtemp )
    local b1, b2, b3 = unpackvec( __vtemp2 )

    setunpacked( __vtemp3, a2 * b3 - a3 * b2, a3 * b1 - a1 * b3, a1 * b2 - a2 * b1 )

	norm(__vtemp3)

	return __vtemp3
end

-- Computes the intersection between a ray and a single triangle
-- Returns the hit position and its Barycentric coordinates
local __eptemp = Vector()

RayIntersectTriangle = function( tris, i, p, dir )
	local A = tris[i].pos
	local B = tris[i+1].pos
	local C = tris[i+2].pos

	local N = TriNormal(A,B,C)

	if ( dot(N, dir) > 0 ) then 
		return false 
	end

	local check, t = RayIntersectPlane( N, A, p, dir ) -- Compute plane intersection against plane triangle sits on

	if ( not check ) then 
		return false 
	end

    set( __eptemp, dir )
    mul( __eptemp, t )
    add( __eptemp, p )
    local ep = __eptemp

    local u, v, w = TriBarycentric( A, B, C, ep ) -- Compute Barycentric coordinates of hit point

    if ( u == 0 and v == 0 and w == 0 ) then -- If Barycentric coordinates rest outside of triangle, it's not a hit
    	return false 
    end

    if ( u < 0 or u > 1 or v < 0 or v > 1 or w < 0 or w > 1 ) then 
    	return false 
    end

    return true, __eptemp, u, v, w

end

RayIntersectTriangles = function( tris, p, dir )
    local dist = math.huge
    local hit, pos, hit_i, u, v, w = false

	for i = #tris - 2, 1, -3 do
		local _hit, _pos, _u, _v, _w = RayIntersectTriangle( tris, i, p, dir )

		if ( _hit ) then
            hit = true

            local _dist = distsqr(_pos, p) 

            if ( _dist < dist ) then
                pos, hit_i, u, v, w = _pos, i, _u, _v, _w
                dist = _dist
            end
        end

	end

    if ( hit ) then 
    	pos = Vector( pos ) 
    end

	return hit, pos, hit_i, u, v, w 
end

local __vmultemp0, __vmultemp1, __vmultemp2, __vmulsum = Vector(), Vector(), Vector(), Vector()

local function BarycentricMul( v0, v1, v2, u, v, w )
    set( __vmultemp0, v0 )
    mul( __vmultemp0, u )
    set( __vmultemp1, v1 )
    mul( __vmultemp1, v )
    set( __vmultemp2, v2 )
    mul( __vmultemp2, w )
    set( __vmulsum, __vmultemp0 )
    add( __vmulsum, __vmultemp1 )
    add( __vmulsum, __vmultemp2 )
    return __vmulsum
end

local __vdist = Vector()

local Ang0 = Angle()

local VisTrace = {}
VisTrace.__index = VisTrace
VisTrace.Mesh = {}

local CurIter = 0

local function ComputeLocalOffset( vtex, entity )
	if ( vtex.localPos ) then return end
	local StudioPose = entity.BindPose[ vtex.weights[1].bone ]
	vtex.localPos, vtex.localAng = WorldToLocal( vtex.pos, vtex.normal:Angle(), StudioPose.pos, StudioPose.ang )
end

local function SkinVertex( vtex, entity )
	if ( vtex.lastiter == CurIter ) then return end
	local BoneMatrix = entity:GetBoneMatrix( vtex.weights[1].bone )

	if ( not BoneMatrix ) then return end

	local pos, ang = LocalToWorld( vtex.localPos, vtex.localAng, BoneMatrix:GetTranslation(), BoneMatrix:GetAngles() ) 
	vtex.pos:Set( pos )
	vtex.normal:Set( ang:Forward() )
	vtex.lastiter = CurIter
end

function VisTrace:Init()
    self.result = { HitNormal = Vector(), HitTexCoord = { 0,0 } }
    self.mtx = Matrix()
    self.imtx = Matrix()
    self.old_pos = Vector()
    self.old_ang = Vector()
    self.matrices_valid = false
end

function VisTrace:SetEntity( entity, LOD )
	if ( not IsValid( entity ) ) then return end

	local model = entity:GetModel()
	local ModelData = util.GetModelMeshes( model, LOD )

	self.Entity = entity

	for num, meshtab in ipairs( ModelData ) do
		for _, vtex in ipairs( meshtab.triangles ) do
			ComputeLocalOffset( vtex, self.Entity )
		end
	end

	self.Mesh = ModelData

	self.BoneNum = entity:GetBoneCount()
end

function VisTrace:UpdateMatrices()

    if ( not IsValid( self.Entity ) ) then 
    	return 
    end

    local pos, ang = self.Entity:GetPos(), self.Entity:GetAngles()

    if ( pos ~= self.old_pos or ang ~= self.old_ang ) then

        self.mtx:SetTranslation( pos )
        self.mtx:SetAngles( ang )
        self.imtx:SetTranslation( pos )
        self.imtx:SetAngles( ang )
        self.matrices_valid = self.imtx:Invert()

        self.old_pos = pos
        self.old_ang = ang

    end

end

function VisTrace:Trace( origin, dir )
	CurIter = ( CurIter % 200 ) + 1

	for num, meshtab in ipairs( self.Mesh ) do
		for _, vtex in ipairs( meshtab.triangles ) do
			SkinVertex( vtex, self.Entity )
		end
	end

    self:UpdateMatrices()

    self.result.Hit = false

    if not self.matrices_valid then return self.result end

    local res = self.result

    origin = self.imtx * origin
	dir = TransformNormal( self.imtx, dir )

    local best_distance = math.huge

    for k, m in ipairs( self.Mesh ) do

        local tris = m.triangles
        local hit, pos, i, u, v, w = RayIntersectTriangles(tris, origin, dir)

        if ( hit ) then
            set( __vdist, pos )
            sub( __vdist, origin )
            local distance = dot( __vdist, dir )

            if ( distance <= best_distance ) then
                local v0 = tris[i]
                local v1 = tris[i+1]
                local v2 = tris[i+2]

                local vnormal = BarycentricMul( v0.normal, v1.normal, v2.normal, u, v, w)
                res.Entity = self.entity
                res.Hit = true
                res.Distance = distance
                res.HitPos = pos
                res.HitNormal = vnormal
                res.HitTexCoord[1] = v0.u * u + v1.u * v + v2.u * w
                res.HitTexCoord[2] = v0.v * u + v1.v * v + v2.v * w
                res.HitMesh = k
                res.HitTri = i
                best_distance = distance
            end

        end

    end

    if res.Hit then
        res.HitPos = self.mtx * res.HitPos
        res.HitNormal = TransformNormal( self.mtx, res.HitNormal )
    end

    return res

end

local c = Color( 100, 100, 255, 255 )

function VisTrace:DrawHitTriangle( res )
    --cam.PushModelMatrix( self.mtx )

    local tris = self.Mesh[res.HitMesh].triangles
    local i = res.HitTri

    local v0 = tris[i]
    local v1 = tris[i+1]
    local v2 = tris[i+2]

    render.DrawLine( v0.pos, v1.pos, c )
    render.DrawLine( v1.pos, v2.pos, c )
    render.DrawLine( v2.pos, v0.pos, c )

    --cam.PopModelMatrix()
end

VisTrace:Init()

if ENABLE_DEMONSTRATION then

    local uv_material = CreateMaterial("uv_mat2", "UnLitGeneric", {
        ["$vertexcolor"] = "1",
        ["$vertexalpha"] = "0",
    })

    local Mdl = "models/player/combine_soldier.mdl"

	if ( IsValid( TRACECLENT ) ) then TRACECLENT:Remove() end

	TRACECLENT = ClientsideModel( Mdl )

	local LOD = 0

	TRACECLENT:SetLOD( LOD )
	TRACECLENT:Spawn()
	TRACECLENT:ResetSequence( -2 ) --Black magic

	timer.Simple( 0, function()
		TRACECLENT:SetupBones()
		TRACECLENT.BindPose = {}

		for i = 0, TRACECLENT:GetBoneCount() - 1 do
			local mat = TRACECLENT:GetBoneMatrix( i )

			if ( mat ) then
				TRACECLENT.BindPose[i] = { pos = mat:GetTranslation(), ang = mat:GetAngles() }
			end
		end

		TRACECLENT:ResetSequence( "walk_all" )

		VisTrace:SetEntity( TRACECLENT, LOD )

		--[[local TS = SysTime()

		for i = 1,100 do
			VisTrace:Trace( EyePos(), EyeVector() )
		end

		print( "avg:", ( SysTime() - TS ) / 100 )]]--
		--avg:	0.032077881406717
	end)

	local Ind = -3

    local function Render()
        local tr = VisTrace:Trace( EyePos(), EyeVector() )

        if input.IsShiftDown() then
        	Ind = Ind + 1
        	TRACECLENT:ResetSequence( Ind )
        end

        if input.IsMouseDown( 107 ) then
        	 hook.Remove("PostDrawOpaqueRenderables", "supertrace")
        end

        if not tr.Hit then return end
 
        VisTrace:DrawHitTriangle( tr )
    end

    hook.Add("PostDrawOpaqueRenderables", "supertrace", Render)
else
    hook.Remove("PostDrawOpaqueRenderables", "supertrace")
end