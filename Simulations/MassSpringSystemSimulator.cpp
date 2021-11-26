#include "MassSpringSystemSimulator.h"

Vec3 tempPos[MAX_MASS];


MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	m_fMass = 10;
	m_fStiffness = 2000;
	m_fDamping = 100;
	m_iIntegrator = 0;
	m_bGrav = FALSE;
	m_bDamping = FALSE;
	m_externalForce = Vec3();

	m_bStarted = FALSE;

	m_iMasses = 0;
	m_iSprings = 0;
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Basic,Mesh,Free Mesh";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler,Leap-Frog,Midpoint");
	TwAddVarRW(DUC->g_pTweakBar,"Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0 step=1");
	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0 step=1");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0 step=1");
	TwAddVarRW(DUC ->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &m_bGrav, "");
	TwAddVarRW(DUC->g_pTweakBar, "Damping?", TW_TYPE_BOOLCPP, &m_bDamping, "");
	TwAddVarRO(DUC->g_pTweakBar, "Number of Masses", TW_TYPE_INT16, &m_iMasses, "");
	TwAddVarRO(DUC->g_pTweakBar, "Number of Springs", TW_TYPE_INT16, &m_iSprings, "");
	//TwAddButton(DUC->g_pTweakBar, "Add Mass Point", [](void* s) { addRandomFreePoint;  }, nullptr, "");
	//TwAddButton(DUC->g_pTweakBar, "Add Fixed Point", [](void* s) { addRandomFixedPoint;  }, nullptr, "");
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, .7, .7), 100, 0.6 * Vec3(1, 1, 1));
	for (int i = 0; i < m_iMasses; ++i) 
	{
		DUC->drawSphere(m_masses[i].pos, Vec3(0.02, 0.02, 0.02));
	}
	DUC->beginLine();
	for (int i = 0; i < m_iSprings; ++i)
	{
		DUC->drawLine(m_springs[i].m_1->pos, Vec3(1, 1, 1), m_springs[i].m_2->pos, Vec3(1, 1, 1));
	}
	DUC->endLine();

}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Basic!\n";
		break;
	case 1:
		cout << "Mesh!\n";
		break;
	case 2:
		cout << "Free Mesh!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
	ResetSim(testCase);
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Vec3* m_vfMovableObjectPos = &m_masses[0].pos;
	Vec3 m_vfMovableObjectFinalPos = m_masses[0].pos;

	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.00001f;
		inputWorld = inputWorld * inputScale;
		*m_vfMovableObjectPos = (m_vfMovableObjectFinalPos + inputWorld);
	}
	else {
		m_vfMovableObjectFinalPos = *m_vfMovableObjectPos;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	float halfStep = timeStep / 2;
	switch (m_iIntegrator)
	{
	case 0:
		eulerStep(timeStep);
		break;
	case 1:
		calculateForces();

		for (int i = 0; i < m_iMasses; ++i)
		{
			m_masses[i].force.safeDivide(m_fMass);
			m_masses[i].acc = m_masses[i].force;
			m_masses[i].acc += m_externalForce;
			if (m_bGrav)
				m_masses[i].acc += GRAV;
			if (!m_bStarted)
			{
				m_masses[i].vel += m_masses[i].acc * (halfStep);
				m_bStarted = TRUE;
			}
			m_masses[i].vel += m_masses[i].acc * timeStep;
			m_masses[i].force = 0;
			if (!m_masses[i].fixed)
				m_masses[i].pos += m_masses[i].vel * timeStep;
		}

		break;
	case 2:
		calculateForces();

		//Update pos by half-step
		//Update acc 
		//Add gravity
		//Update velocity by half-step
		//Clear forces
		for (int i = 0; i < m_iMasses; ++i)
		{
			m_masses[i].tempPos = m_masses[i].pos;
			if (!m_masses[i].fixed)
				m_masses[i].pos += m_masses[i].vel * halfStep;

			m_masses[i].force.safeDivide(m_fMass);
			m_masses[i].acc = m_masses[i].force;
			m_masses[i].acc += m_externalForce;
			if (m_bGrav)
				m_masses[i].acc += GRAV;
			m_masses[i].tempVel = m_masses[i].vel;
			m_masses[i].vel += m_masses[i].acc * halfStep;
			m_masses[i].force = 0;
		}

		calculateForces();

		//Update pos
		//Update acc 
		//Add gravity
		//Update velocity
		//Clear forces
		for (int i = 0; i < m_iMasses; ++i)
		{
			if (!m_masses[i].fixed)
				m_masses[i].pos = m_masses[i].tempPos + m_masses[i].tempVel * timeStep;

			m_masses[i].force.safeDivide(m_fMass);
			m_masses[i].acc = m_masses[i].force;
			m_masses[i].acc += m_externalForce;
			if (m_bGrav)
				m_masses[i].acc += GRAV;
			m_masses[i].vel = m_masses[i].tempVel + m_masses[i].acc * timeStep;
			m_masses[i].force = 0;
		}

		break;
	default:
		break;
	}
	
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::ResetSim(int testCase)
{
	std::mt19937 eng;
	std::uniform_real<float> randY(-0.1f, 0.1f);
	int p;
	switch (testCase)
	{
	case 0:
		m_iMasses = 0;
		m_iSprings = 0;

		addMassPoint(Vec3(0, 1, 0), Vec3(0, 0, 0), TRUE);
		addMassPoint(Vec3(1, 0, 0), Vec3(0, 0, 0), FALSE);
		addMassPoint(Vec3(-1, 0, 1), Vec3(0, 0, 0), FALSE);
		addSpring(0, 1, 1);
		addSpring(0, 2, 1);
		break;
	case 1:
		m_iMasses = 0;
		m_iSprings = 0;
		addMassPoint(Vec3(-1, 0, -1), Vec3(0, 0, 0), TRUE);
		for (float z = -0.9; z < 1; z += 0.2)
		{
			p = addMassPoint(Vec3(-1, 0, z), Vec3(0, 0, 0), TRUE);
			addSpring(p, p - 1, 0.05);
		}

		for (float x = -0.9; x < 1; x += 0.2)
		{
			p = addMassPoint(Vec3(x, 0, -1), Vec3(0, 0, 0), TRUE);
			addSpring(p, p - 11, .05);
			for (float z = -0.9; z < 1; z += 0.2)
			{
				p = addMassPoint(Vec3(x, randY(eng), z), Vec3(0, 0, 0), FALSE);
				addSpring(p, p - 1, 0.05);
				addSpring(p, p - 11, 0.05);
			}

		}
		break;
	case 2:
		m_iMasses = 0;
		m_iSprings = 0;

		addMassPoint(Vec3(-1, 0, -1), Vec3(0, 0, 0), FALSE);
		for (float z = -0.8; z < 1; z += 0.2)
		{
			p = addMassPoint(Vec3(-1, 0, z), Vec3(0, 0, 0), FALSE);
			addSpring(p, p - 1, 0.05);
		}

		for (float x = -0.8; x < 1; x += 0.2)
		{
			p = addMassPoint(Vec3(x, 0, -1), Vec3(0, 0, 0), FALSE);
			addSpring(p, p - 11, .05);
			addSpring(p, p - 10, .05);
			for (float z = -0.8; z < 0.8; z += 0.2)
			{
				p = addMassPoint(Vec3(x, randY(eng), z), Vec3(0, 0, 0), FALSE);
				addSpring(p, p - 1, 0.05);
				addSpring(p, p - 11, 0.05);
				addSpring(p, p - 10, .05);
				addSpring(p, p - 12, .05);
			}
			p = addMassPoint(Vec3(x, 0, 1), Vec3(0, 0, 0), FALSE);
			addSpring(p, p - 1, 0.05);
			addSpring(p, p - 11, .05);
			addSpring(p, p - 12, .05);
		}
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	if (m_iMasses == MAX_MASS)
	{
		cout << "Max number of masses reached!";
		return m_iMasses;
	}

		
	m_masses[m_iMasses].pos = position;
	tempPos[m_iMasses] = position;
	m_masses[m_iMasses].fixed = isFixed;
	if(!isFixed)
		m_masses[m_iMasses].vel = Velocity;
	
	m_masses[m_iMasses].acc = 0;
	m_masses[m_iMasses].force = 0;
	m_iMasses++;
	return m_iMasses - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	m_springs[m_iSprings].m_1 = &m_masses[masspoint1];
	m_springs[m_iSprings].m_2 = &m_masses[masspoint2];
	m_springs[m_iSprings].iLength = initialLength;
	m_iSprings++;
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_iMasses;
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_iSprings;
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_masses[index].pos;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_masses[index].vel;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce += force;
}

void MassSpringSystemSimulator::eulerStep(float timeStep)
{
	calculateForces();

	//Update pos
	//Update acc 
	//Add gravity
	//Update velocity
	//Clear forces
	for (int i = 0; i < m_iMasses; ++i)
	{
		if (!m_masses[i].fixed)
			m_masses[i].pos += m_masses[i].vel * timeStep;

		m_masses[i].force.safeDivide(m_fMass);
		m_masses[i].acc = m_masses[i].force;
		m_masses[i].acc += m_externalForce;
		if (m_bGrav)
			m_masses[i].acc += GRAV;
		m_masses[i].vel += m_masses[i].acc * timeStep;
		m_masses[i].force = 0;
	}
}

void MassSpringSystemSimulator::calculateForces()
{
	//Calculate forces
	//Sum up forces
	for (int i = 0; i < m_iSprings; ++i)
	{
		Vec3 dir = m_springs[i].m_1->pos - m_springs[i].m_2->pos;
		float length = norm(dir);
		dir.safeDivide(length);
		Vec3 force = -m_fStiffness * (length - m_springs[i].iLength) * dir;
		if (m_bDamping)
		{
			m_springs[i].m_1->force -= m_fDamping * m_springs[i].m_1->vel;
			m_springs[i].m_2->force -= m_fDamping * m_springs[i].m_2->vel;
		}
		m_springs[i].m_1->force += force;
		m_springs[i].m_2->force -= force;
	}
}
