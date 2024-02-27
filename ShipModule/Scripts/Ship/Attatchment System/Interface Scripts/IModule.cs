using static UnityEngine.UIElements.UxmlAttributeDescription;
using Unity.VisualScripting;
using UnityEngine;
using System.Collections.Generic;
/// <summary>
/// Base interface for all ship parts. <br/> Ship parts are known as modules.
/// </summary>
public interface IModule
{
    /// <summary>
    /// Retrieves the associated GameObject of this instance.
    /// </summary>
    /// <remarks>
    /// This property should return the GameObject to which the MonoBehaviour (or derived class) is attached.
    /// </remarks>
    public GameObject GetGameObject { get; }

    /// <summary>
    /// The name of the module.
    /// </summary>
    public string Name { get; set; }

    /// <summary>
    /// The remaining hitpoints of the module.
    /// </summary> 
    public int Health { get; set; }

    /// <summary>
    /// The maximum hitpoints of the module.
    /// </summary> 
    public int MaxHealth { get; }

    /// <summary>
    /// List of all objects attached to this module.
    /// </summary>
    public List<IAttachable> AttachedModules { get; set; }
}
