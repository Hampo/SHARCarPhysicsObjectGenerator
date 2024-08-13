using NetP3DLib.P3D.Chunks;
using SHARCarPhysicsObjectGenerator;
using System.Reflection;

if (args.Length == 0 || args.Contains("--help") || args.Contains("-h"))
{
    PrintHelp();
    Console.WriteLine("Press any key to exit . . .");
    Console.ReadKey(true);
    return;
}

List<string> options = [];
string? inputPath = null;
string? outputPath = null;
foreach (string arg in args)
{
    if (!string.IsNullOrWhiteSpace(outputPath))
    {
        Console.WriteLine($"Unknown/unused argument: {arg}");
        continue;
    }

    if (!string.IsNullOrWhiteSpace(inputPath))
    {
        outputPath = arg;
        continue;
    }

    if (arg.StartsWith('-'))
    {
        options.Add(arg[1..]);
        continue;
    }

    inputPath = arg;
}

if (string.IsNullOrWhiteSpace(inputPath))
{
    Console.WriteLine("No input path specified.");
    PrintHelp();
    Console.WriteLine("Press any key to exit . . .");
    Console.ReadKey(true);
    return;
}

bool force = false;
bool noHistory = false;
bool noRemove = false;
foreach (var option in options)
{
    switch (option)
    {
        case "f":
        case "-force":
            force = true;
            break;
        case "nh":
        case "-no_history":
            noHistory = true;
            break;
        case "nr":
        case "-no_remove":
            noRemove = true;
            break;
        default:
            Console.WriteLine($"Unknown/unused option: {option}");
            break;
    }
}

var inputFileInfo = new FileInfo(inputPath);
if (!inputFileInfo.Exists)
{
    Console.WriteLine($"Could not find input path: {inputPath}");
    Console.WriteLine("Press any key to exit . . .");
    Console.ReadKey(true);
    return;
}
inputPath = inputFileInfo.FullName;

if (string.IsNullOrWhiteSpace(outputPath))
{
    outputPath = inputPath;
}

var outputFileInfo = new FileInfo(outputPath);
if (!IsValidOutputPath(outputFileInfo.FullName))
{
    Console.WriteLine("Press any key to exit . . .");
    Console.ReadKey(true);
    return;
}
if (outputFileInfo.Exists && outputFileInfo.IsReadOnly)
{
    Console.WriteLine($"Output path \"{outputFileInfo.FullName}\" is read only.");
    Console.WriteLine("Press any key to exit . . .");
    Console.ReadKey(true);
    return;
}
if (outputFileInfo.Exists && !force)
{
    string? response;
    do
    {
        Console.WriteLine($"Output file \"{outputFileInfo.FullName}\" already exists. Do you want to overwrite? [Yes/No]");
        response = Console.ReadLine();
        if (response != null && response.Equals("no", StringComparison.OrdinalIgnoreCase))
            return;
    } while (response?.ToLower() != "yes");
}
outputPath = outputFileInfo.FullName;

if (!Path.GetExtension(inputPath).Equals(".p3d", StringComparison.OrdinalIgnoreCase))
{
    Console.WriteLine("Input must be a P3D file.");
    Console.WriteLine("Press any key to exit . . .");
    Console.ReadKey(true);
    return;
}

if (!Path.GetExtension(outputPath).Equals(".p3d", StringComparison.OrdinalIgnoreCase))
{
    Console.WriteLine("Output must be a P3D file.");
    Console.WriteLine("Press any key to exit . . .");
    Console.ReadKey(true);
    return;
}

Console.WriteLine($"Input Path: {inputPath}.");
Console.WriteLine($"Output Path: {outputPath}.");
Console.WriteLine($"Force: {force}.");
Console.WriteLine($"No History: {noHistory}.");
Console.WriteLine($"No Remove: {noRemove}.");

try
{
    NetP3DLib.P3D.P3DFile car = new(inputPath);

    var compositeDrawables = car.GetChunksOfType<CompositeDrawableChunk>();
    if (compositeDrawables.Length == 0)
    {
        Console.WriteLine($"Could not find any Composite Drawable chunks in file.");
        return;
    }

    CompositeDrawableChunk? compositeDrawable = null;
    if (compositeDrawables.Length == 1)
    {
        compositeDrawable = compositeDrawables[0];
    }
    else
    {
        string[] compositeDrawableNames = compositeDrawables.Select(x => x.Name).ToArray();
        while (compositeDrawable == null)
        {
            Console.WriteLine("Multiple composite drawables found. Please pick from the following list:");
            for (int i = 0; i < compositeDrawableNames.Length; i++)
                Console.WriteLine($"\t[{i}] {compositeDrawableNames[i]}");
            string? indexStr = Console.ReadLine();
            if (!int.TryParse(indexStr, out var index) || index < 0 || index >= compositeDrawableNames.Length)
            {
                Console.WriteLine($"Invalid index specified. Please enter an index between 0 and {compositeDrawableNames.Length - 1}.");
            }
            else
            {
                compositeDrawable = compositeDrawables[index];
            }
        }
    }
    var carName = compositeDrawable.Name;
    var skeletonName = compositeDrawable.SkeletonName;

    var skeleton = car.GetFirstChunkOfType<SkeletonChunk>(skeletonName);
    if (skeleton == null)
    {
        Console.WriteLine($"Could not find Skeleton chunk with name: {skeletonName}.");
        return;
    }
    var skeletonBV = car.GetFirstChunkOfType<SkeletonChunk>($"{skeletonName}BV") ?? skeleton;

    var collisionObject = car.GetFirstChunkOfType<CollisionObjectChunk>(skeletonName);
    if (skeleton == null)
    {
        Console.WriteLine($"Could not find Skeleton chunk with name: {skeletonName}.");
        return;
    }
    var collisionObjectBV = car.GetFirstChunkOfType<CollisionObjectChunk>($"{skeletonName}BV");

    var physicsObject = car.GetFirstChunkOfType<PhysicsObjectChunk>(carName);
    if (physicsObject != null)
    {
        if (noRemove)
        {
            Console.WriteLine("Renaming exiting Physics Object.");
            physicsObject.Name += "_old";
        }
        else
        {
            Console.WriteLine("Deleting exiting Physics Object.");
            car.Chunks.Remove(physicsObject);
        }
    }
    Console.WriteLine("Generating new Physics Object.");
    physicsObject = PhysicsObjectGenerator.GeneratePhysicsObject(skeleton, collisionObject);

    var physicsObjectBV = car.GetFirstChunkOfType<PhysicsObjectChunk>($"{carName}BV");
    if (physicsObjectBV != null)
    {
        if (noRemove)
        {
            Console.WriteLine("Renaming exiting Physics Object BV.");
            physicsObjectBV.Name += "_old";
        }
        else
        {
            Console.WriteLine("Deleting exiting Physics Object BV.");
            car.Chunks.Remove(physicsObjectBV);
        }
    }
    if (collisionObjectBV == null)
    {
        Console.WriteLine("Not generating Physics Object BV due to no Collision Object BV.");
        physicsObjectBV = null;
    }
    else
    {
        Console.WriteLine("Generating new Physics Object BV.");
        physicsObjectBV = PhysicsObjectGenerator.GeneratePhysicsObject(skeletonBV, collisionObjectBV, true);
    }

    if (!noHistory)
        car.Chunks.Add(new HistoryChunk([$"Physics Objects Generated by SHARCarPhysicsObjectGenerator v{Assembly.GetExecutingAssembly().GetName().Version}."]));
    car.Chunks.Add(physicsObject);
    if (physicsObjectBV != null)
        car.Chunks.Add(physicsObjectBV);

    car.Write(outputPath);

    Console.WriteLine($"Saved updated P3D file to: {outputPath}");
}
catch (Exception ex)
{
    Console.WriteLine($"There was an error generating the Physics Object: {ex}");
    Console.WriteLine("Press any key to exit . . .");
    Console.ReadKey(true);
}

static void PrintHelp()
{
    Console.WriteLine("Usage: SHARCarPhysicsObjectGenerator [options] <input_path> [output_path]");
    Console.WriteLine();
    Console.WriteLine("Options:");
    Console.WriteLine("  -f  | --force         Force overwrite the output file.");
    Console.WriteLine("  -nh | --no_history    Don't add history chunk.");
    Console.WriteLine("  -nr | --no_remove     Don't remove existing PhysicsObject chunks, they will renamed instead.");
    Console.WriteLine();
    Console.WriteLine("Arguments:");
    Console.WriteLine("  <input_path>   The input car's P3D file.");
    Console.WriteLine("  [output_path]  The output P3D file. If omitted, it will attempt to overwrite \"input_path\".");
    Console.WriteLine();
    Console.WriteLine("Example:");
    Console.WriteLine("  SHARCarPhysicsObjectGenerator C:\\input\\car.p3d C:\\output\\car.p3d");
    Console.WriteLine("  SHARCarPhysicsObjectGenerator --force --no_history C:\\input\\car.p3d");
    Console.WriteLine();
}

static bool IsValidOutputPath(string outputPath)
{
    if (outputPath.IndexOfAny(Path.GetInvalidPathChars()) != -1)
    {
        Console.WriteLine($"Output path \"{outputPath}\" contains invalid characters.");
        return false;
    }

    var directory = Path.GetDirectoryName(outputPath);
    if (!Directory.Exists(directory))
    {
        Console.WriteLine($"Output directory \"{(string.IsNullOrWhiteSpace(directory), Environment.CurrentDirectory, directory)}\" doesn't exist.");
        return false;
    }

    try
    {
        var path = Path.GetRandomFileName();
        if (!string.IsNullOrWhiteSpace(directory))
            path = Path.Combine(directory, path);
        using FileStream fs = File.Create(path, 1, FileOptions.DeleteOnClose);
    }
    catch
    {
        return false;
    }

    return true;
}