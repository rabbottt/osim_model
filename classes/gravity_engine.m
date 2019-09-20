classdef gravity_engine < handle
    
properties (Access = private)
    id_ % ID of the session
end

methods
    function this = gravity_engine()
        %GravityEngine Create a new
        this.id_ = GravityEngine('new');
    end
    
    function delete(this)
        %delete Destructor
        GravityEngine('delete', this.id_);
    end
    
    function setup(this, fileName)
        assert(isscalar(this));
        assert(ischar(fileName));
        GravityEngine('setup', this.id_, fileName)
    end
    
    function updateState(this, stateVec)
        assert(isscalar(this));
        GravityEngine('updateState', this.id_, stateVec);
    end

    function newQ = getStateCoords(this)
        assert(isscalar(this));
        newQ = GravityEngine('getStateCoords', this.id_);
    end
    
    function gravForces = calcGravForces(this)
        assert(isscalar(this));
        gravForces = GravityEngine('calcGravForces', this.id_);
    end
    
end
end

    
        
    