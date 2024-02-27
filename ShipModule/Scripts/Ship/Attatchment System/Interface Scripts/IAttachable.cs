using UnityEngine;
/// <summary>
/// Base interface for all modules which can be attached to attachment points.
/// </summary>
public interface IAttachable : IModule
{
    /// <summary>
    /// True if the module is attached to an attachment point.
    /// </summary>
    public bool IsAttached { get; }

    /// <summary>
    /// Attempts to attach the current module to a specified attachment point on a parent module.
    /// </summary>
    /// <remarks>
    /// On successful attachment, this function will:
    /// <list type="bullet">
    /// <item>Align the current module's game object with the game object of the target attachment point.</item>
    /// <item>Set the current module's game object as a child of the parent module's game object. <br/> The parent module's game object is retrieved from the target attachment point.</item>
    /// <item>Change the current module's game object rigidbody to kinematic.</item>
    /// <item>Set the attachment flag to true.</item>
    /// </list>
    /// </remarks> 
    public void Attach(AttachmentPoint attachmentPoint);

    /// <summary>
    /// Ejects the current module from its parent module.
    /// </summary>
    /// <remarks>
    /// On successful ejection, this function will:
    /// <list type="bullet">
    /// <item>Unset the current module's game object as a child of the parent module's game object.</item>
    /// <item>Change the current module's game object rigidbody to dynamic.</item>
    /// <item>Set the attachment flag to false.</item>
    /// <item>Apply a force to the current module's game object to push it away from the parent module.</item>
    /// <item>Get the player ship's current vector and slowly apply it to the current module's game object to halt its movement relative to the player ship.</item>
    /// </list>
    /// </remarks>
    public void Eject();

    /// <summary>
    /// Disconnects the current module from its parent module.
    /// </summary>
    /// <remarks>
    /// On successful disconnection, this function will:
    /// <list type="bullet">
    /// <item><description>Unset the current module's game object as a child of the parent module's game object.</description></item>
    /// <item><description>Disable the rigidbody interactions of the current module's game object.</description></item>
    /// <item><description>Set the attachment flag to false.</description></item>
    /// </list>
    /// The rigidbody interactions are disabled to ensure the module, post-disconnection, does not collide with other objects while being constrained to the player's mouse cursor. This ensures that the player cannot use the module to interact physically with the environment or other objects.
    /// </remarks>
    public void Disconnect(bool removeFromList = true);


    /// <summary>
    /// Prepares the module to be picked up by the player.
    /// </summary>
    public void PickUp();

    /// <summary>
    /// Reverts the module to its original state after being picked up by the player and applies a velocity to the module.
    /// </summary>
    public void Release(Vector3 releaseVelocity);

    /// <summary>
    /// Returns the first found attachment point on the current module with ConnectionPoint set to true.
    /// </summary>
    public AttachmentPoint FindThisConnectionPoint();
}
