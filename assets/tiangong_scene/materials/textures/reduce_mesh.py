import bpy
import os

def reduce_mesh_quality(obj, ratio=0.5):
    """Reduce the mesh quality of the given object by the given ratio."""
    if obj.type == 'MESH':
        # Select the object
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        
        # Enter edit mode
        bpy.ops.object.mode_set(mode='EDIT')
        
        # Select all mesh elements
        bpy.ops.mesh.select_all(action='SELECT')
        
        # Reduce mesh quality
        bpy.ops.mesh.decimate(ratio=ratio)
        
        # Return to object mode
        bpy.ops.object.mode_set(mode='OBJECT')
        
        # Deselect the object
        obj.select_set(False)
    else:
        print(f"Object {obj.name} is not a mesh, skipping reduction.")

def import_and_reduce(filepath, ratio=0.5):
    """Import a file and reduce its mesh quality."""
    ext = os.path.splitext(filepath)[1].lower()
    
    # Import the file based on its extension
    if ext == '.stl':
        bpy.ops.import_mesh.stl(filepath=filepath)
    elif ext == '.dae':
        bpy.ops.wm.collada_import(filepath=filepath)
    else:
        print(f"Unsupported file type: {ext}")
        return
    
    # Reduce the quality of all imported objects
    for obj in bpy.context.selected_objects:
        reduce_mesh_quality(obj, ratio)
    
    # Export the file back to its original location
    if ext == '.stl':
        bpy.ops.export_mesh.stl(filepath=filepath, use_selection=True)
    elif ext == '.dae':
        bpy.ops.wm.collada_export(filepath=filepath, selected=True)

def main(directory, ratio=0.5):
    """Main function to process all .stl and .dae files in the given directory."""
    for filename in os.listdir(directory):
        if filename.lower().endswith(('.stl', '.dae')):
            filepath = os.path.join(directory, filename)
            import_and_reduce(filepath, ratio)
            print(f"Processed {filepath}")

# Set the directory to the current working directory
directory = os.getcwd()

# Set the reduction ratio (0.5 means 50% reduction in quality)
reduction_ratio = 0.5

# Run the main function
main(directory, reduction_ratio)

