# AGENTS.md

## Cursor Cloud specific instructions

### Project Overview

CADability is a .NET 3D CAD system comprising a core library (`netstandard2.0`), a Windows Forms UI layer (`net48`), and a thin desktop app shell (`net48`). See `README.md` for details.

### Build

Build the core library and tests (excludes `CADability.DebuggerVisualizers` which requires Visual Studio):

```bash
dotnet build CADability/CADability.csproj -p:EnableWindowsTargeting=true
dotnet build CADability.Forms/CADability.Forms.csproj -p:EnableWindowsTargeting=true
dotnet build tests/CADability.Tests/CADability.Tests.csproj -p:EnableWindowsTargeting=true
```

The `-p:EnableWindowsTargeting=true` flag is required on Linux to cross-compile `net6.0-windows` and `net48` targets.

Do **not** build the full solution (`dotnet build CADability.sln`) on Linux — the `CADability.DebuggerVisualizers` project will fail because it depends on `Microsoft.VisualStudio.DebuggerVisualizers.dll`.

### Tests

```bash
dotnet test tests/CADability.Tests/CADability.Tests.csproj -p:EnableWindowsTargeting=true
```

Expected results on Linux: **10 pass / 5 fail**. The 5 failures are platform-specific:
- 2 tests fail with `System.Drawing.Common is not supported on this platform` (bitmap comparison uses GDI+ which is Windows-only in .NET 6).
- 2 tests fail with DXF code page errors (Windows code page 850/1252 not available on Linux).
- 1 test fails due to MSTest `DeploymentItem` file copy issue on Linux.

These are not regressions — they are expected on non-Windows platforms.

### Fake WindowsDesktop Runtime

To run `net6.0-windows` tests on Linux, a fake `Microsoft.WindowsDesktop.App` runtime directory is created at `/usr/share/dotnet/shared/Microsoft.WindowsDesktop.App/6.0.36/` by copying assemblies from `Microsoft.NETCore.App`. This is a one-time setup step. After a fresh VM, if tests fail with "Framework 'Microsoft.WindowsDesktop.App' ... not found", re-run:

```bash
sudo mkdir -p /usr/share/dotnet/shared/Microsoft.WindowsDesktop.App/6.0.36
sudo cp /usr/share/dotnet/shared/Microsoft.NETCore.App/6.0.36/*.dll /usr/share/dotnet/shared/Microsoft.WindowsDesktop.App/6.0.36/
sudo cp /usr/share/dotnet/shared/Microsoft.NETCore.App/6.0.36/*.so /usr/share/dotnet/shared/Microsoft.WindowsDesktop.App/6.0.36/
```

Also copy `System.Drawing.Common.dll` to the test output so bitmap-related tests can at least attempt to run:

```bash
cp ~/.nuget/packages/system.drawing.common/6.0.0/lib/net6.0/System.Drawing.Common.dll tests/CADability.Tests/bin/Debug/net6.0-windows/
```

### Key Gotchas

- The `Microsoft.VisualStudio.DebuggerVisualizers` reference produces a warning (MSB3245) on every build. This is harmless — the types are only used at debug-time in Visual Studio.
- `CADability.App` and `CADability.Forms` target `net48` and cannot run as desktop apps on Linux (no Windows Forms runtime). The core library and tests are the primary development targets on this platform.
- `libgdiplus` must be installed for any `System.Drawing.Common` usage on Linux: `sudo apt-get install -y libgdiplus`.
