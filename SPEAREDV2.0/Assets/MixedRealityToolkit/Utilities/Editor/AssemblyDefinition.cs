// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using System;
using System.IO;
using UnityEngine;

namespace Microsoft.MixedReality.Toolkit.Utilities.Editor
{
    /// <summary>
    /// A class that represents a Unity assembly definition (asmdef) file.
    /// </summary>
    [Serializable]
    public class AssemblyDefinition
    {
        /// <summary>
        /// Creates a new, empty assembly definition.
        /// </summary>
        public AssemblyDefinition()
        { }

        [SerializeField]
        private string name = null;

        /// <summary>
        /// Please see <see href="https://docs.unity3d.com/Manual/class-AssemblyDefinitionImporter.html">Assembly Definition properties</see> on the Unity documentation site.
        /// </summary>
        public string Name
        {
            get => name;
            set => name = value;
        }

        // General
        [SerializeField]
        private bool allowUnsafeCode = false;

        /// <summary>
        /// Please see <see href="https://docs.unity3d.com/Manual/class-AssemblyDefinitionImporter.html">Assembly Definition properties</see> on the Unity documentation site.
        /// </summary>
        public bool AllowUnsafeCode
        {
            get => allowUnsafeCode;
            set => allowUnsafeCode = value;
        }

        [SerializeField]
        private bool autoReferenced = true;

        /// <summary>
        /// Please see <see href="https://docs.unity3d.com/Manual/class-AssemblyDefinitionImporter.html">Assembly Definition properties</see> on the Unity documentation site.
        /// </summary>
        public bool AutoReferenced
        {
            get => autoReferenced;
            set => autoReferenced = value;
        }

        [SerializeField]
        private bool overrideReferences = false;

        /// <summary>
        /// Please see <see href="https://docs.unity3d.com/Manual/class-AssemblyDefinitionImporter.html">Assembly Definition properties</see> on the Unity documentation site.
        /// </summary>
        public bool OverrideReferences
        {
            get => overrideReferences;
            set => overrideReferences = value;
        }

        // Define constraints
        [SerializeField]
        private string[] defineConstraints = null;

        /// <summary>
        /// Please see <see href="https://docs.unity3d.com/Manual/class-AssemblyDefinitionImporter.html">Assembly Definition properties</see> on the Unity documentation site.
        /// </summary>
        public string[] DefineConstraints
        {
            get => defineConstraints;
            set => defineConstraints = value;
        }

        // Assembly references
        [SerializeField]
        private string[] references = null;

        /// <summary>
        /// Please see <see href="https://docs.unity3d.com/Manual/class-AssemblyDefinitionImporter.html">Assembly Definition properties</see> on the Unity documentation site.
        /// </summary>
        public string[] References
        {
            get => references;
            set => references = value;
        }

        // Unity references
        [SerializeField]
        private string[] optionalUnityReferences = null;

        /// <summary>
        /// Please see <see href="https://docs.unity3d.com/Manual/class-AssemblyDefinitionImporter.html">Assembly Definition properties</see> on the Unity documentation site.
        /// </summary>
        public string[] OptionalUnityReferences
        {
            get => optionalUnityReferences;
            set => optionalUnityReferences = value;
        }

        // Platforms
        [SerializeField]
        private string[] includePlatforms = null;

        /// <summary>
        /// Please see <see href="https://docs.unity3d.com/Manual/class-AssemblyDefinitionImporter.html">Assembly Definition properties</see> on the Unity documentation site.
        /// </summary>
        public string[] IncludePlatforms
        {
            get => includePlatforms;
            set => includePlatforms = value;
        }

        [SerializeField]
        private string[] excludePlatforms = null;

        /// <summary>
        /// Please see <see href="https://docs.unity3d.com/Manual/class-AssemblyDefinitionImporter.html">Assembly Definition properties</see> on the Unity documentation site.
        /// </summary>
        public string[] ExcludePlatforms
        {
            get => excludePlatforms;
            set => excludePlatforms = value;
        }

        /// <summary>
        /// Loads an existing assembly definition file.
        /// </summary>
        /// <param name="fileName">The file to be loaded.</param>
        /// <returns>The assembly definition that has been loaded, or null.</returns>
        public static AssemblyDefinition Load(string fileName)
        {
            if (string.IsNullOrWhiteSpace(fileName))
            {
                Debug.LogError("An assembly definition file name must be specified.");
                return null;
            }

            FileInfo file = new FileInfo(fileName);
            if (!file.Exists)
            {
                Debug.LogError($"The {fileName} file could not be found.");
                return null;
            }

            return JsonUtility.FromJson<AssemblyDefinition>(File.ReadAllText(file.FullName));
        }

        /// <summary>
        /// Saves an assembly definition file.
        /// </summary>
        /// <param name="fileName">The name by which to save the assembly definition file.</param>
        /// <remarks>
        /// If the specified file exists, it will be overwritten.
        /// </remarks>
        public void Save(string fileName)
        {
            if (string.IsNullOrWhiteSpace(fileName))
            {
                Debug.LogError("A name for the assembly definition file must be specified.");
                return;
            }

            FileInfo file = new FileInfo(fileName);
            bool readOnly = file.IsReadOnly;
            if (readOnly)
            {
                file.IsReadOnly = false;
            }

            Debug.Log($"Saving {fileName}");
            File.WriteAllText(fileName, JsonUtility.ToJson(this, true));

            if (readOnly)
            {
                file.IsReadOnly = true;
            }
        }
    }
}
