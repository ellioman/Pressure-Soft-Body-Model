using UnityEngine;
using UnityEngine.Assertions;
using System.Collections;

[RequireComponent(typeof(LineRenderer))]
public class PressureSoftBody : MonoBehaviour
{
	#region Variables

	// Unity Editor Variables
	[SerializeField] [Range(0, 0.5f)] protected float particleColliderRadius;// The size of the particle colliders
	[SerializeField] protected int numOfParticles;							// How many particles should the body have?
	[SerializeField] protected float bodyRadius;							// How big should the body be initially?
	[SerializeField] protected int iterationsPerFixedUpdate;				// How many iterations should we calculate within each FixedUpdate?
	[SerializeField] protected float pressure;								// The amount of pressure inside the body
	[SerializeField] protected float mass;									// The Mass to set on the particle's rigidbody
	[SerializeField] protected float rigidbodyDrag;							// The Drag to set on the particle's rigidbody
	[SerializeField] protected float springConstantElasticity;				// spring constant
	[SerializeField] protected float springDamping;							// spring damping constant
	[SerializeField] protected float gravityScale;							// The factor to multiply the gravity vector

	// Protected Instance Variables
	protected float deltaTime = 0f;											// deltaTime    => Time.fixedDeltaTime / iterationsPerFixedUpdate
	protected LineRenderer lineRend = null;									// The renderer we use to show the body
	protected PSBParticle[] particles = null;								// Container for all particles used
	protected PSBParticle[] predictedParticles = null;						// Temporary container for Heun Integration
	protected PSBInternalSpring[] internalSprings = null;					// All Springs inside the body
	protected PolygonCollider2D polyCollider = null;						// The polygon collider which we use to get initial particle positions

	#endregion
	

	#region Monobehaviour

	// Constructor
	protected void Awake()
	{
		lineRend = GetComponent<LineRenderer>();
		Assert.IsNotNull(lineRend);
	}

	// Use this for initialization
	protected void Start()
	{
		// Make sure we can draw the entire body with the line renderer...
		lineRend.SetVertexCount(numOfParticles + 1);
		
		// Create the particles used for the PSB model
		CreateParticlesAndSprings();
	}
	
	// Called every fixed framerate frame
	protected void FixedUpdate()
	{
		// ------------------------------------------------------------------
		// STEP 1: Update the particles array with the data from Unity's Rigidbodies...
		// ------------------------------------------------------------------
		for (int i = 0; i < numOfParticles; i++)
		{
			particles[i].position = particles[i].rBody.position;
			particles[i].velocity = particles[i].rBody.velocity;
		}
		
		// ------------------------------------------------------------------
		// STEP 2: Calculate the Pressure Soft Body forces using Heun integration
		// ------------------------------------------------------------------
		deltaTime = Time.fixedDeltaTime / iterationsPerFixedUpdate;
		for (int i = 0; i < iterationsPerFixedUpdate; i++)
		{
			AccumulateForces(ref particles);
			IntegrateHeunPredictor();
			AccumulateForces(ref predictedParticles);
			IntegrateHeunCorrector();
		}
		
		// ------------------------------------------------------------------
		// STEP 3: Move the particles based on the PSB Calculations...
		// ------------------------------------------------------------------
		for (int i = 0; i < numOfParticles ; i++)
		{
			particles[i].rBody.MovePosition(particles[i].position);
		}
	}
	
	// Update is called once per frame
	protected void Update()
	{
		// ------------------------------------------------------------------
		// Check Input...
		// ------------------------------------------------------------------

		// Reset position of the body...
		if (Input.GetKeyUp(KeyCode.Space))
		{
			// Calculate the center of body
			Vector2 centerPos = Vector2.zero;
			for (int i = 0; i < numOfParticles ; i++)
			{
				centerPos += particles[i].position;
			}
			centerPos /= numOfParticles;

			// Put the body back to (0,0)
			for (int i = 0; i < numOfParticles ; i++)
			{
				particles[i].rBody.position = particles[i].rBody.position - centerPos;
				particles[i].rBody.velocity = Vector2.zero;
			}
		}


		// ------------------------------------------------------------------
		// Draw the body...
		// ------------------------------------------------------------------

		for (int i = 0; i < numOfParticles ; i++)
		{
			Debug.DrawLine(particles[i].position, particles[(i + 1) % numOfParticles].position, Color.red);

			lineRend.SetPosition(i, particles[i].rBody.position);

		}
		lineRend.SetPosition(numOfParticles, particles[0].rBody.position);
	}
	
	#endregion


	#region Pressure Soft Body Calculations....
	
	// Calculates the accumululation of forces to the character
	protected void AccumulateForces(ref PSBParticle[] particleArray)
	{
		float lengthBetweenParticles = 0f;	// length of p1 - p2 vector
		float hookeForceValue = 0f;			// hooke force value
		float pressurev = 0f;				// pressure force value
		float volume = 0f;					// the volume of the body
		Vector2 particle1Pos = Vector2.zero;// positions of spring particles p1, p2
		Vector2 particle2Pos = Vector2.zero;// positions of spring particles p1, p2
		Vector2 v12 = Vector2.zero;			// particle1Pos.vel - particle2Pos.vel
		Vector2 force = Vector2.zero;		// the calculated force for each spring

		// Reset all forces
		for (int i = 0; i < numOfParticles ; i++)
		{
			particleArray[i].force = Vector2.zero;
		}

		/* spring force */
		for (int i = 0; i < numOfParticles ; i++)
		{
			// get positions of spring start & end particles
			particle1Pos = particleArray[internalSprings[i].i].position; 
			particle2Pos = particleArray[internalSprings[i].j].position; 
			lengthBetweenParticles = (particle1Pos - particle2Pos).magnitude; // calculate square root  of the distance
			
			if (lengthBetweenParticles != 0)
			{
				// get velocities of start & end particles
				v12 = particleArray[internalSprings[i].i].velocity - particleArray[internalSprings[i].j].velocity;
				
				// calculate force value
				hookeForceValue = (lengthBetweenParticles - internalSprings[i].length) * springConstantElasticity + (v12.x * (particle1Pos.x - particle2Pos.x) + v12.y * (particle1Pos.y - particle2Pos.y)) * springDamping / lengthBetweenParticles;
				
				// force vector
				force = ((particle1Pos - particle2Pos) / lengthBetweenParticles ) * hookeForceValue;
				
				// accumulate force for starting particle
				particleArray[internalSprings[i].i].force -= force;
				
				// accumulate force for end particle
				particleArray[internalSprings[i].j].force += force;
			}
			
			// Calculate normal vectors to springs 
			internalSprings[i].norm.x =  (particle1Pos.y - particle2Pos.y) / lengthBetweenParticles;
			internalSprings[i].norm.y = -(particle1Pos.x - particle2Pos.x) / lengthBetweenParticles;
		}
		
		/* pressure force */
		
		/* Calculate Volume of the Ball (Gauss Theorem) */
		for (int i = 0; i < numOfParticles; i++)
		{
			particle1Pos = particleArray[internalSprings[i].i].position;
			particle2Pos = particleArray[internalSprings[i].j].position;
			lengthBetweenParticles = (particle1Pos - particle2Pos).magnitude;
			volume += 0.5f * Mathf.Abs(particle1Pos.x - particle2Pos.x) * Mathf.Abs(internalSprings[i].norm.x) * (lengthBetweenParticles);
		}
		
		// Calculate Pressure Force Distribution
		for (int i = 0; i < numOfParticles; i++)
		{
			// Get the particles and calculate the distance between them
			particle1Pos = particleArray[internalSprings[i].i].position;
			particle2Pos = particleArray[internalSprings[i].j].position;	
			lengthBetweenParticles = (particle1Pos - particle2Pos).magnitude;
			
			// P = 1/V * A * P * N
			pressurev = lengthBetweenParticles * pressure * (1.0f/volume);
			particleArray[internalSprings[i].i].force.x += internalSprings[i].norm.x * pressurev;
			particleArray[internalSprings[i].i].force.y += internalSprings[i].norm.y * pressurev;
			particleArray[internalSprings[i].j].force.x += internalSprings[i].norm.x * pressurev;
			particleArray[internalSprings[i].j].force.y += internalSprings[i].norm.y * pressurev;
			
			// Calculate the particle normal by using the normal of the two springs attached to it
			particleArray[i].normal = (internalSprings[particleArray[i].prevInternalSpring].norm + internalSprings[particleArray[i].nextInternalSpring].norm).normalized;
		}
	}
	
	// Heun Predictor: Predicts the position and velocity of the particles...
	// Euler Integrator (second Newton's law)
	protected void IntegrateHeunPredictor()
	{
		for (int i = 0 ; i < numOfParticles; i++)
		{
			predictedParticles[i].velocity = particles[i].velocity + (particles[i].force / mass) * deltaTime; // + Acceleration * deltatime...
			predictedParticles[i].position = particles[i].position + predictedParticles[i].velocity * deltaTime;
		}
	}
	
	// Heun Corrector: Corrects the velocity and position
	protected void IntegrateHeunCorrector()
	{
		for (int i = 0; i < numOfParticles; i++)
		{
			particles[i].velocity = particles[i].velocity + (deltaTime/2.0f) * (particles[i].force + predictedParticles[i].force) / mass;
			particles[i].position += particles[i].velocity * deltaTime;
		}
	}
	
	// Creates the particles and internal springs for the body
	protected void CreateParticlesAndSprings()
	{
		CreateParticles();
		CreateInternalSprings();
	}
	
	// Creates particles for the body
	protected void CreateParticles()
	{
		particles = new PSBParticle[numOfParticles];
		predictedParticles = new PSBParticle[numOfParticles];
		internalSprings = new PSBInternalSpring[numOfParticles];
		
		// Create the particles
		for (int i = 0; i < numOfParticles; i++)
		{
			particles[i] = new PSBParticle();
			predictedParticles[i] = new PSBParticle();

			particles[i].position.x = bodyRadius * Mathf.Sin((i + 1) * (2.0f * Mathf.PI) / numOfParticles);
			particles[i].position.y = bodyRadius * Mathf.Cos((i + 1) * (2.0f * Mathf.PI) / numOfParticles);// + SCRSIZE/2;

			// Gameobject
			particles[i].gameObj = new GameObject();
			particles[i].gameObj.name = "Particle_" + i;
			particles[i].gameObj.transform.parent = transform;
			particles[i].gameObj.transform.localPosition = Vector2.zero;
			particles[i].gameObj.layer = 8;
			
			// Rigidbody
			particles[i].rBody = particles[i].gameObj.AddComponent<Rigidbody2D>();
			particles[i].rBody.mass = mass;
			particles[i].rBody.position = particles[i].position;
			particles[i].rBody.angularDrag = 0.5f;
			particles[i].rBody.drag = rigidbodyDrag;
			particles[i].rBody.constraints = RigidbodyConstraints2D.FreezeRotation;
			particles[i].rBody.collisionDetectionMode = CollisionDetectionMode2D.Continuous;
			particles[i].rBody.gravityScale = gravityScale;
			
			// Collider
			particles[i].circleCol = particles[i].gameObj.AddComponent<CircleCollider2D>();
			particles[i].circleCol.radius = particleColliderRadius;
		}
	}
	
	// Creates the internal springs for the body
	protected void CreateInternalSprings()
	{
		for (int i = 0; i < numOfParticles; i++)
		{
			AddInternalSpring(i, i, ((i + 1) % numOfParticles));
		}
	}
	
	// Adds an internal spring to the character
	protected void AddInternalSpring(int springIndex, int particle1Index, int particle2Index)
	{
		internalSprings[springIndex] = new PSBInternalSpring();
		internalSprings[springIndex].i = particle1Index;
		particles[particle1Index].nextInternalSpring = springIndex;
		predictedParticles[particle1Index].nextInternalSpring = springIndex;
		
		internalSprings[springIndex].j = particle2Index; 
		particles[particle2Index].prevInternalSpring = springIndex;
		predictedParticles[particle2Index].prevInternalSpring = springIndex;
		
		internalSprings[springIndex].length = Mathf.Sqrt(
			(particles[particle1Index].position.x - particles[particle2Index].position.x) * 
			(particles[particle1Index].position.x - particles[particle2Index].position.x) + 
			(particles[particle1Index].position.y - particles[particle2Index].position.y) * 
			(particles[particle1Index].position.y - particles[particle2Index].position.y)
			);
	}
	
	#endregion
}
