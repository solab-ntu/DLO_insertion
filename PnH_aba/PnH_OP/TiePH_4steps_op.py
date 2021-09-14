from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
from odbAccess import *
from abaqusConstants import *
from odbMaterial import *
from odbSection import *
import regionToolset
import numpy as np
import json
# import GA
# import geneticalgorithm as ga


        
class RunModel:
    def __init__(self):
        with open('PnH_aba/PnH_OP/SimCondOP.json', 'r') as f:
            SimSet = json.load(f)
        self.iteration = SimSet['iteration']
        self.iter_to_Insert = SimSet['iter_to_Insert']
        # if iteration > the iteration to insert
        # set the model to dual step mode
        if self.iteration >= self.iter_to_Insert:
            self.dual = True
        else:
            self.dual = False
        self.tot_rot = 5
        self.step1Time = 0.001/3
        self.step2Time = 0.001
        self.overrot = SimSet['OverRot']
        self.rotbac = SimSet['RotBac']

    def simula(self):
        # Build the model
        PnH.modelBuilding()
        # step
        PnH.stepSet()
        # interaction
        PnH.interactionSet()
        # amplitude
        PnH.AmpSet()
        # boundary condition for movement
        PnH.bondary_conditionSet()
        # predefined field
        PnH.predefined_field()
        # job setting
        PnH.jobSet()
        # job submitment
        PnH.jobSubmitment()

    def modelBuilding(self):        # don't edit
        mdb.Model(name='Model-'+str(self.iteration), modelType=STANDARD_EXPLICIT)
        #############
        # PEG part
        #############
        mdb.models['Model-'+str(self.iteration)].ConstrainedSketch(name='__profile__', sheetSize=200.0)
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].ConstructionLine(point1=(0.0, 
            -100.0), point2=(0.0, 100.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].FixedConstraint(entity=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[2])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].Line(point1=(0.0, -1.0), point2=(
            0.5, -1.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].HorizontalConstraint(
            addUndoState=False, entity=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[3])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].Line(point1=(0.5, -1.0), point2=
            (0.5, 39.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].VerticalConstraint(addUndoState=
            False, entity=mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[4])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].PerpendicularConstraint(
            addUndoState=False, entity1=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[3], entity2=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[4])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].Line(point1=(0.5, 39.0), point2=
            (0.0, 39.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].HorizontalConstraint(
            addUndoState=False, entity=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[5])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].PerpendicularConstraint(
            addUndoState=False, entity1=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[4], entity2=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[5])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].CoincidentConstraint(
            addUndoState=False, entity1=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].vertices[3], entity2=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[2])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].Line(point1=(0.0, 39.0), point2=(
            0.0, -1.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].VerticalConstraint(addUndoState=
            False, entity=mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[6])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].PerpendicularConstraint(
            addUndoState=False, entity1=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[5], entity2=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[6])
        mdb.models['Model-'+str(self.iteration)].Part(dimensionality=THREE_D, name='PEG', type=
            DEFORMABLE_BODY)
        mdb.models['Model-'+str(self.iteration)].parts['PEG'].BaseSolidRevolve(angle=360.0, 
            flipRevolveDirection=OFF, sketch=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'])
        del mdb.models['Model-'+str(self.iteration)].sketches['__profile__']
        # PEG partition
        mdb.models['Model-'+str(self.iteration)].parts['PEG'].DatumPlaneByPrincipalPlane(offset=38.0, 
            principalPlane=XZPLANE)
        mdb.models['Model-'+str(self.iteration)].parts['PEG'].PartitionCellByDatumPlane(cells=
            mdb.models['Model-'+str(self.iteration)].parts['PEG'].cells.getSequenceFromMask(('[#1 ]', ), )
            , datumPlane=mdb.models['Model-'+str(self.iteration)].parts['PEG'].datums[2])
        #############
        # HOLE part
        #############
        mdb.models['Model-'+str(self.iteration)].ConstrainedSketch(name='__profile__', sheetSize=200.0)
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].ConstructionLine(point1=(0.0, 
            -100.0), point2=(0.0, 100.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].FixedConstraint(entity=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[2])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].Line(point1=(0.0, -39.0), point2=
            (0.58, -39.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].HorizontalConstraint(
            addUndoState=False, entity=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[3])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].Line(point1=(0.58, -39.0), 
            point2=(0.58, 0.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].VerticalConstraint(addUndoState=
            False, entity=mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[4])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].PerpendicularConstraint(
            addUndoState=False, entity1=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[3], entity2=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[4])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].Line(point1=(0.58, 0.0), point2=
            (5.0, 0.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].HorizontalConstraint(
            addUndoState=False, entity=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[5])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].PerpendicularConstraint(
            addUndoState=False, entity1=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[4], entity2=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[5])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].Line(point1=(5.0, 0.0), point2=(
            5.0, -40.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].VerticalConstraint(addUndoState=
            False, entity=mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[6])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].PerpendicularConstraint(
            addUndoState=False, entity1=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[5], entity2=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[6])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].Line(point1=(5.0, -40.0), point2=
            (0.0, -40.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].HorizontalConstraint(
            addUndoState=False, entity=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[7])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].PerpendicularConstraint(
            addUndoState=False, entity1=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[6], entity2=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[7])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].CoincidentConstraint(
            addUndoState=False, entity1=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].vertices[5], entity2=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[2])
        mdb.models['Model-'+str(self.iteration)].Part(dimensionality=THREE_D, name='HOLE', type=
            ANALYTIC_RIGID_SURFACE)
        mdb.models['Model-'+str(self.iteration)].parts['HOLE'].AnalyticRigidSurfRevolve(sketch=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'])
        del mdb.models['Model-'+str(self.iteration)].sketches['__profile__']
        # HOLE reference point
        mdb.models['Model-'+str(self.iteration)].parts['HOLE'].ReferencePoint(point=
            mdb.models['Model-'+str(self.iteration)].parts['HOLE'].InterestingPoint(
            mdb.models['Model-'+str(self.iteration)].parts['HOLE'].edges[3], CENTER))
        #############
        # CLAMP part
        #############
        mdb.models['Model-'+str(self.iteration)].ConstrainedSketch(name='__profile__', sheetSize=200.0)
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].ConstructionLine(point1=(0.0, 
            -100.0), point2=(0.0, 100.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].FixedConstraint(entity=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[2])
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].Line(point1=(1.0, 38.0), point2=(
            1.0, 39.0))
        mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].VerticalConstraint(addUndoState=
            False, entity=mdb.models['Model-'+str(self.iteration)].sketches['__profile__'].geometry[3])
        mdb.models['Model-'+str(self.iteration)].Part(dimensionality=THREE_D, name='CLAMP', type=
            DISCRETE_RIGID_SURFACE)
        mdb.models['Model-'+str(self.iteration)].parts['CLAMP'].BaseShellRevolve(angle=360.0, 
            flipRevolveDirection=OFF, sketch=
            mdb.models['Model-'+str(self.iteration)].sketches['__profile__'])
        del mdb.models['Model-'+str(self.iteration)].sketches['__profile__']
        # CLAMP reference point & inertia
        mdb.models['Model-'+str(self.iteration)].parts['CLAMP'].ReferencePoint(point=
            mdb.models['Model-'+str(self.iteration)].parts['CLAMP'].InterestingPoint(
            mdb.models['Model-'+str(self.iteration)].parts['CLAMP'].edges[1], CENTER))
        mdb.models['Model-'+str(self.iteration)].parts['CLAMP'].engineeringFeatures.PointMassInertia(
            alpha=0.0, composite=0.0, mass=0.001, name='MAD_Inertia', region=Region(
            referencePoints=(mdb.models['Model-'+str(self.iteration)].parts['CLAMP'].referencePoints[2], 
            )))
        #############
        # PEG mesh
        #############
        mdb.models['Model-'+str(self.iteration)].parts['PEG'].seedPart(deviationFactor=0.1, minSizeFactor=
            0.1, size=0.7)
        mdb.models['Model-'+str(self.iteration)].parts['PEG'].generateMesh()
        #############
        # CLAMP mesh
        #############
        mdb.models['Model-'+str(self.iteration)].parts['CLAMP'].seedPart(deviationFactor=0.1, 
            minSizeFactor=0.1, size=0.2)
        mdb.models['Model-'+str(self.iteration)].parts['CLAMP'].generateMesh()
        #############
        # material
        #############
        mdb.models['Model-'+str(self.iteration)].Material(name='Steel')
        mdb.models['Model-'+str(self.iteration)].materials['Steel'].Density(table=((7.8e-09, ), ))
        mdb.models['Model-'+str(self.iteration)].materials['Steel'].Elastic(table=((210000.0, 0.28), ))
        mdb.models['Model-'+str(self.iteration)].materials['Steel'].Plastic(table=((380.0, 0.0), (420.0, 
            0.04), (470.0, 0.12), (500.0, 0.19), (522.0, 0.25)))
        #############
        # section
        #############
        mdb.models['Model-'+str(self.iteration)].HomogeneousSolidSection(material='Steel', name=
            'Steel_Section', thickness =None)
        #############
        # PEG section assignment
        #############
        mdb.models['Model-'+str(self.iteration)].parts['PEG'].SectionAssignment(offset=0.0, offsetField=''
            , offsetType=MIDDLE_SURFACE, region=Region(
            cells=mdb.models['Model-'+str(self.iteration)].parts['PEG'].cells.getSequenceFromMask(mask=(
            '[#3 ]', ), )), sectionName='Steel_Section', thicknessAssignment=
            FROM_SECTION)
        '''
        #############
        # surface
        #############
        mdb.models['Model-'+str(self.iteration)].parts['PEG'].Surface(name='Slave', side1Faces=
            mdb.models['Model-'+str(self.iteration)].parts['PEG'].faces.getSequenceFromMask(('[#8 ]', ), 
            ))
        mdb.models['Model-'+str(self.iteration)].parts['CLAMP'].Surface(name='Master', side1Faces=
            mdb.models['Model-'+str(self.iteration)].parts['CLAMP'].faces.getSequenceFromMask(('[#1 ]', 
            ), ))
        mdb.models['Model-'+str(self.iteration)].parts['HOLE'].Surface(name='HOLE_surf', side1Faces=
            mdb.models['Model-'+str(self.iteration)].parts['HOLE'].faces.getSequenceFromMask(('[#1f ]', ), 
            ))
        '''

        #############
        # assembly
        #############
        mdb.models['Model-'+str(self.iteration)].rootAssembly.DatumCsysByDefault(CARTESIAN)
        mdb.models['Model-'+str(self.iteration)].rootAssembly.Instance(dependent=ON, name='HOLE-1', part=
            mdb.models['Model-'+str(self.iteration)].parts['HOLE'])
        mdb.models['Model-'+str(self.iteration)].rootAssembly.Instance(dependent=ON, name='CLAMP-1', 
            part=mdb.models['Model-'+str(self.iteration)].parts['CLAMP'])
        mdb.models['Model-'+str(self.iteration)].rootAssembly.Instance(dependent=ON, name='PEG-1', part=
            mdb.models['Model-'+str(self.iteration)].parts['PEG'])

        mdb.models['Model-'+str(self.iteration)].rootAssembly.Surface(name='Slave', side1Faces=
            mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['PEG-1'].faces.getSequenceFromMask(
            ('[#8 ]', ), ))
        mdb.models['Model-'+str(self.iteration)].rootAssembly.Surface(name='Master', side1Faces=
            mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['CLAMP-1'].faces.getSequenceFromMask(
            ('[#1 ]', ), ))
        mdb.models['Model-'+str(self.iteration)].rootAssembly.Surface(name='HOLE_surface', side1Faces=
            mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['HOLE-1'].faces.getSequenceFromMask(
            ('[#1f ]', ), ))
        #############
        # Constraints
        #############
        a = mdb.models['Model-'+str(self.iteration)].rootAssembly
        f1 = a.instances['CLAMP-1'].faces
        faces1 = f1.getSequenceFromMask(mask=('[#1 ]', ), )
        region2=regionToolset.Region(faces=faces1)
        a = mdb.models['Model-'+str(self.iteration)].rootAssembly
        f1 = a.instances['PEG-1'].faces
        faces1 = f1.getSequenceFromMask(mask=('[#8 ]', ), )
        region4=regionToolset.Region(faces=faces1)
        a = mdb.models['Model-'+str(self.iteration)].rootAssembly
        r1 = a.instances['CLAMP-1'].referencePoints
        refPoints1=(r1[2], )
        region1=regionToolset.Region(referencePoints=refPoints1)
        mdb.models['Model-'+str(self.iteration)].RigidBody(name='Rigid', refPointRegion=region1, 
            bodyRegion=region2, tieRegion=region4)

    def AmpSet(self):
        mdb.models['Model-'+str(self.iteration)].TabularAmplitude(data=((0.0, 0.0), (self.step1Time, 1.0)), name=
            'Amp-1', smooth=SOLVER_DEFAULT, timeSpan=STEP)
        mdb.models['Model-'+str(self.iteration)].TabularAmplitude(data=((0.0, 0.0), (self.step2Time, 1.0)), name=
            'Amp-2', smooth=SOLVER_DEFAULT, timeSpan=STEP)
        return None

    def stepSet(self):
        if self.dual:
            mdb.models['Model-'+str(self.iteration)].ExplicitDynamicsStep(name='Rotating1', previous='Initial', 
                timePeriod=self.step1Time, improvedDtMethod=ON)
            for i in range(1,self.tot_rot):
                if i <= 2:
                    mdb.models['Model-'+str(self.iteration)].ExplicitDynamicsStep(name=('Rotating'+str(i+1)), previous=('Rotating'+str(i)), 
                        timePeriod=self.step1Time/3, improvedDtMethod=ON)
                else:
                    mdb.models['Model-'+str(self.iteration)].ExplicitDynamicsStep(name=('Rotating'+str(i+1)), previous=('Rotating'+str(i)), 
                        timePeriod=self.step1Time/2, improvedDtMethod=ON)
            mdb.models['Model-'+str(self.iteration)].ExplicitDynamicsStep(name='Inserting', previous=('Rotating'+str(self.tot_rot)), 
                timePeriod=self.step2Time, improvedDtMethod=ON)

        else:
            mdb.models['Model-'+str(self.iteration)].ExplicitDynamicsStep(name='Inserting', previous='Initial', 
                timePeriod=self.step2Time, improvedDtMethod=ON)
        return self.step1Time,self.step2Time

    def interactionSet(self):  # don't edit
        if self.dual:
            for i in range(1,self.tot_rot+1):
                mdb.models['Model-'+str(self.iteration)].ContactProperty('FrictionR'+str(i))
                mdb.models['Model-'+str(self.iteration)].interactionProperties['FrictionR'+str(i)].TangentialBehavior(
                    dependencies=0, directionality=ISOTROPIC, elasticSlipStiffness=None, 
                    formulation=PENALTY, fraction=0.005, maximumElasticSlip=FRACTION, 
                    pressureDependency=OFF, shearStressLimit=None, slipRateDependency=OFF, 
                    table=((0.3, ), ), temperatureDependency=OFF)
                mdb.models['Model-'+str(self.iteration)].ContactExp(createStepName='Rotating'+str(i), name='ContactR'+str(i))
                mdb.models['Model-'+str(self.iteration)].interactions['ContactR'+str(i)].includedPairs.setValuesInStep(
                    stepName='Rotating'+str(i), useAllstar=ON)
                mdb.models['Model-'+str(self.iteration)].interactions['ContactR'+str(i)].contactPropertyAssignments.appendInStep(
                    assignments=((GLOBAL, SELF, 'FrictionR'+str(i)), ), stepName=('Rotating'+str(i)))
                for j in range(i,self.tot_rot):
                    mdb.models['Model-'+str(self.iteration)].interactions['ContactR'+str(i)].deactivate('Rotating'+str(j+1))



            
        else:
            mdb.models['Model-'+str(self.iteration)].ContactProperty('Friction')
            mdb.models['Model-'+str(self.iteration)].interactionProperties['Friction'].TangentialBehavior(
                dependencies=0, directionality=ISOTROPIC, elasticSlipStiffness=None, 
                formulation=PENALTY, fraction=0.005, maximumElasticSlip=FRACTION, 
                pressureDependency=OFF, shearStressLimit=None, slipRateDependency=OFF, 
                table=((0.3, ), ), temperatureDependency=OFF)

            mdb.models['Model-'+str(self.iteration)].ContactExp(createStepName='Inserting', name='Contact')
            mdb.models['Model-'+str(self.iteration)].interactions['Contact'].includedPairs.setValuesInStep(
                stepName='Inserting', useAllstar=ON)
            mdb.models['Model-'+str(self.iteration)].interactions['Contact'].contactPropertyAssignments.appendInStep(
                assignments=((GLOBAL, SELF, 'Friction'), ), stepName='Inserting')        

    def bondary_conditionSet(self):
        if self.dual:
            for i in range(1,self.tot_rot+1):
                mdb.models['Model-'+str(self.iteration)].EncastreBC(createStepName=('Rotating'+str(i)), localCsys=None, 
                    name=('HOLE_fixedR'+str(i)), region=Region(referencePoints=(
                    mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['HOLE-1'].referencePoints[2], 
                    )))     # fixed hole for rotating
                for j in range(i,self.tot_rot):
                    mdb.models['Model-'+str(self.iteration)].boundaryConditions['HOLE_fixedR'+str(i)].deactivate('Rotating'+str(j+1))

            # node 1 = No.508
            # node clamp = No.79
            odb = openOdb(path=('Job'+str(self.iteration-1)+'.odb'))
            myAssembly = odb.rootAssembly
            # last frame displace value
            lastFrame = odb.steps['Inserting'].frames[-1]
            coordinate = lastFrame.fieldOutputs['COORD']
            # print(coordinate)
            coordValues = np.array(coordinate.values)
            # print(coordValues)

            Manicoord = np.array([coordValues[267].data[0],coordValues[267].data[1],coordValues[267].data[2]])
            print(Manicoord)

            P1coord = np.array([coordValues[698-(self.iteration-self.iter_to_Insert)].data[0],
                coordValues[698-(self.iteration-self.iter_to_Insert)].data[1],coordValues[698-(self.iteration-self.iter_to_Insert)].data[2]])
            print(P1coord)

            # rotation matrix construction
            P1Height = np.sqrt(np.square(P1coord[0]) + np.square(P1coord[1]) + np.square(P1coord[2]))
            print(P1Height)
            RotaAxis = np.cross(P1coord,np.array([0,1,0]))
            RotaAxis = RotaAxis/np.sqrt(np.square(RotaAxis[0]) + np.square(RotaAxis[1]) + np.square(RotaAxis[2]))
            print(RotaAxis)
            RotaTheta = np.arccos(np.absolute(P1coord[1]/P1Height))
            a = np.cos(RotaTheta/3/2)
            b = np.sin(RotaTheta/3/2)*RotaAxis[0]
            c = np.sin(RotaTheta/3/2)*RotaAxis[1]
            d = np.sin(RotaTheta/3/2)*RotaAxis[2]
            RotaMat = np.array(
                [[1-2*np.square(c)-2*np.square(d),2*b*c-2*a*d,2*a*c+2*b*d],
                [2*b*c+2*a*d,1-2*np.square(b)-2*np.square(d),2*c*d-2*a*b],
                [2*b*d-2*a*c,2*a*b+2*c*d,1-2*np.square(b)-2*np.square(c)]])

            a_o = np.cos(RotaTheta/3/2/self.overrot)
            b_o = np.sin(RotaTheta/3/2/self.overrot)*RotaAxis[0]
            c_o = np.sin(RotaTheta/3/2/self.overrot)*RotaAxis[1]
            d_o = np.sin(RotaTheta/3/2/self.overrot)*RotaAxis[2]
            RotaMat_o = np.array(
                [[1-2*np.square(c_o)-2*np.square(d_o),2*b_o*c_o-2*a_o*d_o,2*a_o*c_o+2*b_o*d_o],
                [2*b_o*c_o+2*a_o*d_o,1-2*np.square(b_o)-2*np.square(d_o),2*c_o*d_o-2*a_o*b_o],
                [2*b_o*d_o-2*a_o*c_o,2*a_o*b_o+2*c_o*d_o,1-2*np.square(b_o)-2*np.square(c_o)]])

            a_b = np.cos(-RotaTheta/3/2/self.rotbac)
            b_b = np.sin(-RotaTheta/3/2/self.rotbac)*RotaAxis[0]
            c_b = np.sin(-RotaTheta/3/2/self.rotbac)*RotaAxis[1]
            d_b = np.sin(-RotaTheta/3/2/self.rotbac)*RotaAxis[2]
            RotaMat_b = np.array(
                [[1-2*np.square(c_b)-2*np.square(d_b),2*b_b*c_b-2*a_b*d_b,2*a_b*c_b+2*b_b*d_b],
                [2*b_b*c_b+2*a_b*d_b,1-2*np.square(b_b)-2*np.square(d_b),2*c_b*d_b-2*a_b*b_b],
                [2*b_b*d_b-2*a_b*c_b,2*a_b*b_b+2*c_b*d_b,1-2*np.square(b_b)-2*np.square(c_b)]])
            # calculate the manipulator's position goal
            P1Goal1 = np.dot(RotaMat,P1coord.T)
            P1Goal2 = np.dot(RotaMat,P1Goal1)
            P1Goal3 = np.dot(RotaMat,P1Goal2)
            print(P1Goal3)
            ManiGoal1 = np.dot(RotaMat,Manicoord.T)
            ManiGoal2 = np.dot(RotaMat,ManiGoal1)
            ManiGoal3 = np.dot(RotaMat,ManiGoal2)
            ManiGoal4 = np.dot(RotaMat_o,ManiGoal3)
            ManiGoal5 = np.dot(RotaMat_b,ManiGoal4)
            print(ManiGoal3)
            # calculate the rotation angle
            if P1coord[2] > 0:
                Alpha = -np.arccos(P1coord[1]/np.sqrt(np.square(P1coord[2]) + np.square(P1coord[1])))/3
            else:
                Alpha = np.arccos(P1coord[1]/np.sqrt(np.square(P1coord[2]) + np.square(P1coord[1])))/3
            print(Alpha)
            
            if P1coord[0] > 0:
                Gamma = np.arccos(P1coord[1]/np.sqrt(np.square(P1coord[0]) + np.square(P1coord[1])))/3
            else:
                Gamma = -np.arccos(P1coord[1]/np.sqrt(np.square(P1coord[0]) + np.square(P1coord[1])))/3
            print(Gamma)
            
            # R1
            mdb.models['Model-'+str(self.iteration)].DisplacementBC(amplitude='Amp-1', createStepName=
                'Rotating1', distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=
                None, name='PEG_maniR1', region=Region(referencePoints=(
                mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['CLAMP-1'].referencePoints[2], 
                )), u1=(ManiGoal1[0]-Manicoord[0]), u2= (ManiGoal1[1]-Manicoord[1]), u3=(ManiGoal1[2]-Manicoord[2]), ur1=Alpha, ur2=0, ur3=Gamma)
            mdb.models['Model-'+str(self.iteration)].boundaryConditions['PEG_maniR1'].deactivate('Rotating2')
            mdb.models['Model-'+str(self.iteration)].boundaryConditions['PEG_maniR1'].deactivate('Rotating3')
            mdb.models['Model-'+str(self.iteration)].boundaryConditions['PEG_maniR1'].deactivate('Rotating4')
            mdb.models['Model-'+str(self.iteration)].boundaryConditions['PEG_maniR1'].deactivate('Rotating5')

            # R2
            mdb.models['Model-'+str(self.iteration)].DisplacementBC(amplitude='Amp-1', createStepName=
                'Rotating2', distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=
                None, name='PEG_maniR2', region=Region(referencePoints=(
                mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['CLAMP-1'].referencePoints[2], 
                )), u1=(ManiGoal2[0]-ManiGoal1[0]), u2= (ManiGoal2[1]-ManiGoal1[1]), u3=(ManiGoal2[2]-ManiGoal1[2]), ur1=Alpha, ur2=0, ur3=Gamma)
            mdb.models['Model-'+str(self.iteration)].boundaryConditions['PEG_maniR2'].deactivate('Rotating3')
            mdb.models['Model-'+str(self.iteration)].boundaryConditions['PEG_maniR2'].deactivate('Rotating4')
            mdb.models['Model-'+str(self.iteration)].boundaryConditions['PEG_maniR2'].deactivate('Rotating5')

            # R3
            mdb.models['Model-'+str(self.iteration)].DisplacementBC(amplitude='Amp-1', createStepName=
                'Rotating3', distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=
                None, name='PEG_maniR3', region=Region(referencePoints=(
                mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['CLAMP-1'].referencePoints[2], 
                )), u1=(ManiGoal3[0]-ManiGoal2[0]), u2= (ManiGoal3[1]-ManiGoal2[1]), u3=(ManiGoal3[2]-ManiGoal2[2]), ur1=Alpha, ur2=0, ur3=Gamma)
            mdb.models['Model-'+str(self.iteration)].boundaryConditions['PEG_maniR3'].deactivate('Rotating4')
            mdb.models['Model-'+str(self.iteration)].boundaryConditions['PEG_maniR3'].deactivate('Rotating5')

            # R4
            mdb.models['Model-'+str(self.iteration)].DisplacementBC(amplitude='Amp-1', createStepName=
                'Rotating4', distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=
                None, name='PEG_maniR4', region=Region(referencePoints=(
                mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['CLAMP-1'].referencePoints[2], 
                )), u1=(ManiGoal4[0]-ManiGoal3[0]), u2= (ManiGoal4[1]-ManiGoal3[1]), u3=(ManiGoal4[2]-ManiGoal3[2]), ur1=Alpha/self.overrot, ur2=0, ur3=Gamma/self.overrot)
            mdb.models['Model-'+str(self.iteration)].boundaryConditions['PEG_maniR4'].deactivate('Rotating5')

            # R5
            mdb.models['Model-'+str(self.iteration)].DisplacementBC(amplitude='Amp-1', createStepName=
                'Rotating5', distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=
                None, name='PEG_maniR5', region=Region(referencePoints=(
                mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['CLAMP-1'].referencePoints[2], 
                )), u1=(ManiGoal5[0]-ManiGoal4[0]), u2=(ManiGoal5[1]-ManiGoal4[1]), u3=(ManiGoal5[2]-ManiGoal4[2]), ur1=-Alpha/self.rotbac, ur2=0, ur3=-Gamma/self.rotbac)



        else:
            mdb.models['Model-'+str(self.iteration)].EncastreBC(createStepName='Inserting', localCsys=None, 
                name='HOLE_fixed', region=Region(referencePoints=(
                mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['HOLE-1'].referencePoints[2], 
                )))     # fixed hole
            mdb.models['Model-'+str(self.iteration)].DisplacementBC(amplitude='Amp-2', createStepName=
                'Inserting', distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=
                None, name='PEG_mani', region=Region(referencePoints=(
                mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['CLAMP-1'].referencePoints[2], 
                )), u1=10.0, u2=-10.0, u3=0.0, ur1=1.0, ur2=-0.5, ur3=0.2)
        return None         
        
    def predefined_field(self):     # don't edit
        # predefined field
        if self.iteration > 1:   # we don't need this funtion in model 1
            # mdb.models['Model-'+str(self.iteration)].InitialState(createStepName='Initial', endIncrement=
            #     STEP_END, endStep=LAST_STEP, fileName='Job'+str(self.iteration-1), instances=(
            #     mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['PEG-1'], ), name=
            #     'Predefined_Field_PEG', updateReferenceConfiguration=ON)
            
            # mdb.models['Model-'+str(self.iteration)].InitialState(createStepName='Initial', endIncrement=
            #     STEP_END, endStep=LAST_STEP, fileName='Job'+str(self.iteration-1), instances=(
            #     mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['CLAMP-1'], ), name=
            #     'Predefined_Field_CLAMP', updateReferenceConfiguration=ON)
            instances=(mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['CLAMP-1'], 
                mdb.models['Model-'+str(self.iteration)].rootAssembly.instances['PEG-1'], )
            mdb.models['Model-'+str(self.iteration)].InitialState(updateReferenceConfiguration=ON, 
                fileName='Job'+str(self.iteration-1), endStep=LAST_STEP, endIncrement=STEP_END, 
                name='Predefined Field-1', createStepName='Initial', 
                instances=instances)

            # a = mdb.models['Model-'+str(self.iteration)].rootAssembly
            # f1 = a.instances['CLAMP-1'].faces
            # faces1 = f1.getSequenceFromMask(mask=('[#1 ]', ), )
            # region2=regionToolset.Region(faces=faces1)
            # a = mdb.models['Model-'+str(self.iteration)].rootAssembly
            # f1 = a.instances['PEG-1'].faces
            # faces1 = f1.getSequenceFromMask(mask=('[#8 ]', ), )
            # region4=regionToolset.Region(faces=faces1)
            # a = mdb.models['Model-'+str(self.iteration)].rootAssembly
            # r1 = a.instances['CLAMP-1'].referencePoints
            # refPoints1=(r1[2], )
            # region1=regionToolset.Region(referencePoints=refPoints1)
            # mdb.models['Model-'+str(self.iteration)].RigidBody(name='Rigid', refPointRegion=region1, 
            #     bodyRegion=region2, tieRegion=region4)
        mdb.models['Model-'+str(self.iteration)].parts['CLAMP'].setValues(space=THREE_D, 
            type=DEFORMABLE_BODY)
        mdb.models['Model-'+str(self.iteration)].Material(name='rigid')
        mdb.models['Model-'+str(self.iteration)].materials['rigid'].Density(table=((7.8e-09, ), ))
        mdb.models['Model-'+str(self.iteration)].materials['rigid'].Elastic(table=((2e+15, 0.0), ))
        mdb.models['Model-'+str(self.iteration)].HomogeneousShellSection(name='Rigid_Section', 
            preIntegrate=OFF, material='rigid', thicknessType=UNIFORM, 
            thickness=0.5, thicknessField='', nodalThicknessField='', 
            idealization=NO_IDEALIZATION, poissonDefinition=DEFAULT, 
            thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, 
            integrationRule=SIMPSON, numIntPts=5)
        p = mdb.models['Model-'+str(self.iteration)].parts['CLAMP']
        f = p.faces
        faces = f.getSequenceFromMask(mask=('[#1 ]', ), )
        region = regionToolset.Region(faces=faces)
        p = mdb.models['Model-'+str(self.iteration)].parts['CLAMP']
        p.SectionAssignment(region=region, sectionName='Rigid_Section', offset=0.0, 
            offsetType=MIDDLE_SURFACE, offsetField='', 
            thicknessAssignment=FROM_SECTION)
        p = mdb.models['Model-'+str(self.iteration)].parts['CLAMP']
        p.generateMesh()
        a = mdb.models['Model-'+str(self.iteration)].rootAssembly
        a.regenerate()
        a = mdb.models['Model-'+str(self.iteration)].rootAssembly
        a.regenerate()
        region3=None
        mdb.models['Model-'+str(self.iteration)].constraints['Rigid'].setValues(pinRegion=region3)

            
        return None

    def jobSet(self):               # don't edit
        mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF, 
            explicitPrecision=SINGLE, historyPrint=OFF, memory=90, memoryUnits=
            PERCENTAGE, model=('Model-'+str(self.iteration)), modelPrint=OFF, name=('JobOP'), 
            nodalOutputPrecision=SINGLE, queue=None, resultsFormat=ODB, scratch='', 
            type=ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)            #job settings
        mdb.models['Model-'+str(self.iteration)].fieldOutputRequests['F-Output-1'].setValues(variables=(
            'S', 'SVAVG', 'PE', 'PEVAVG', 'PEEQ', 'PEEQVAVG', 'LE', 'U', 'V', 'A', 
            'RF', 'CSTRESS', 'EVF', 'COORD','CFORCE'))
        mdb.jobs['JobOP'].setValues(explicitPrecision=DOUBLE_PLUS_PACK)

        return None

    def jobSubmitment(self):
        # job submitment
        # no comment for running the simulation
        mdb.jobs['JobOP'].submit(consistencyChecking=OFF)
        mdb.jobs['JobOP'].waitForCompletion()
        return None

    def getRes(self):
        odb = openOdb(path=('JobOP'+'.odb'))
        myAssembly = odb.rootAssembly
        # last frame displace value
        lastFrame = odb.steps['Rotating5'].frames[-1]
        stress = lastFrame.fieldOutputs['S']
        # get stress mises
        stressValues = np.average([np.absolute(stress.values[177-4*(self.iteration-self.iter_to_Insert)].data[3]), \
            np.absolute(stress.values[177-4*(self.iteration-self.iter_to_Insert)].data[5]), \
            np.absolute(stress.values[180-4*(self.iteration-self.iter_to_Insert)].data[3]), \
            np.absolute(stress.values[180-4*(self.iteration-self.iter_to_Insert)].data[5]), \
            np.absolute(stress.values[178-4*(self.iteration-self.iter_to_Insert)].data[3]), \
            np.absolute(stress.values[178-4*(self.iteration-self.iter_to_Insert)].data[5]), \
            np.absolute(stress.values[179-4*(self.iteration-self.iter_to_Insert)].data[3]), \
            np.absolute(stress.values[179-4*(self.iteration-self.iter_to_Insert)].data[5])] )

        # get displacement
        coordinate_R = lastFrame.fieldOutputs['COORD']
        coordValues_R = np.array(coordinate_R.values)
        # print(coordValues)

        P1coord_R = np.array([coordValues_R[698-(self.iteration-self.iter_to_Insert)].data[0],
            coordValues_R[698-(self.iteration-self.iter_to_Insert)].data[1],coordValues_R[698-(self.iteration-self.iter_to_Insert)].data[2]])
        P1coord_R2 = np.array([coordValues_R[699-(self.iteration-self.iter_to_Insert)].data[0],
            coordValues_R[699-(self.iteration-self.iter_to_Insert)].data[1],coordValues_R[699-(self.iteration-self.iter_to_Insert)].data[2]])
        data_R = {'MisesStress': np.float64(stressValues).item(),\
            'Xcoord': np.float64(P1coord_R[0]-P1coord_R2[0]).item(),\
            'Zcoord': np.float64(P1coord_R[2]-P1coord_R2[2]).item()}
        with open('PnH_aba/PnH_OP/SimRes.json', 'w') as f:
            json.dump(data_R, f)
        


if __name__ == '__main__':
    PnH = RunModel()
    PnH.simula()
    PnH.getRes()
