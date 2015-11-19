using UnityEngine;
using System.Collections;

[System.Serializable]
public class PSBInternalSpring
{
	#region Variables

	public int i = 0;                   // Index to the first particle
	public int j = 0;                   // Index to the second particle
	public float length = 0f;           // rest length
	public Vector2 norm = Vector2.zero; // normal

	#endregion
}
