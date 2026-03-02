# AGENTS.md

## Cursor Cloud specific instructions

### Project Overview

CADability is a .NET CAD library + Windows Forms desktop application. The solution contains:

| Project | Target | Buildable on Linux? |
|---|---|---|
| `CADability` (core) | `netstandard2.0` | Yes |
| `CADability.Forms` | `net48` (WinForms) | Build only (with `-p:EnableWindowsTargeting=true`) |
| `CADability.App` | `net48` (WinExe) | Build only (with `-p:EnableWindowsTargeting=true`) |
| `CADability.Tests` | `net6.0-windows` | Build only; **cannot run** (requires `Microsoft.WindowsDesktop.App` runtime) |
| `netDxf` | `net48;net6.0` | Yes (net6.0 target) |

### Build Commands

- **Core library only (recommended):** `dotnet build CADability/CADability.csproj -c Debug`
- **Tests project (build only):** `dotnet build tests/CADability.Tests/CADability.Tests.csproj -p:EnableWindowsTargeting=true -c Debug`
- **Full solution build not supported** on Linux due to `net48` + WinForms dependencies.

### Key Caveats

1. **Tests cannot run on Linux.** The test project targets `net6.0-windows` with `UseWindowsForms=true` and depends on `CADability.Forms` (net48). The test host requires `Microsoft.WindowsDesktop.App` runtime, which is unavailable on Linux. Tests can only be compiled, not executed.
2. **`Microsoft.VisualStudio.DebuggerVisualizers` reference** in `CADability.csproj` cannot resolve on Linux (warning MSB3245). This is harmless — the build succeeds.
3. **`Text` GeoObject** uses native Win32 `gdi32.dll` P/Invoke. Any code touching `Text.CalcExtent()` or font operations will crash on Linux. Avoid `Text` objects in test/demo code running on Linux.
4. **DXF import** requires `System.Text.Encoding.CodePages`. Call `Encoding.RegisterProvider(CodePagesEncodingProvider.Instance)` before importing DXF files on Linux.
5. **Stale obj directories** can cause "Duplicate assembly attribute" errors when switching between direct core builds and full-solution builds with `EnableWindowsTargeting`. Clean all `obj/` directories if this happens.
6. **No linter configured.** There is no separate lint tool; the C# compiler warnings serve as the lint check. Run `dotnet build` and inspect warnings.
7. **`libgdiplus` must be installed** (`sudo apt-get install -y libgdiplus`) for `System.Drawing.Common` to work on Linux. Required only if code uses bitmap/drawing operations.
