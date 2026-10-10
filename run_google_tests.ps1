param(
    [ValidateSet('Debug', 'Release')][string]$Configuration = 'Debug',
    [ValidateSet('x64', 'Win32')][string]$Platform = 'x64',
    [string]$Filter = '*'
)

$ErrorActionPreference = 'Stop'
$dependency = Join-Path $PSScriptRoot 'third_party\googletest'
$expectedCommit = '063de7e9578f82b369302001269680b4b1553359'
if (!(Test-Path -LiteralPath $dependency)) {
    & git clone --depth 1 --branch v1.18.0 https://github.com/google/googletest.git $dependency
    if ($LASTEXITCODE -ne 0) { throw 'GoogleTest download failed.' }
}
$actualCommit = & git -C $dependency rev-parse HEAD
if ($LASTEXITCODE -ne 0 -or $actualCommit -ne $expectedCommit) {
    throw 'Expected the pinned GoogleTest v1.18.0 checkout. Check third_party\googletest.'
}

$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (!(Test-Path -LiteralPath $vswhere)) { throw 'Visual Studio Installer was not found.' }
$msbuild = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find 'MSBuild\Current\Bin\amd64\MSBuild.exe' | Select-Object -First 1
if (!$msbuild) { throw 'Install Visual Studio Desktop development with C++.' }
& $msbuild (Join-Path $PSScriptRoot 'RvcGoogleTests.vcxproj') /t:Build "/p:Configuration=$Configuration" "/p:Platform=$Platform" /verbosity:minimal /nologo
if ($LASTEXITCODE -ne 0) { throw 'GoogleTest build failed.' }

$resultsDirectory = Join-Path $PSScriptRoot 'test-results'
New-Item -ItemType Directory -Path $resultsDirectory -Force | Out-Null
$xmlPath = Join-Path $resultsDirectory 'google-test-results.xml'
$logPath = Join-Path $resultsDirectory 'google-test-console.txt'
$testExe = Join-Path $PSScriptRoot "build\google-tests\$Platform\$Configuration\bin\rvc_google_tests.exe"
& $testExe "--gtest_filter=$Filter" "--gtest_output=xml:$xmlPath" 2>&1 | Tee-Object -FilePath $logPath
$testExitCode = $LASTEXITCODE
if ($testExitCode -ne 0) { throw "GoogleTest failed (exit $testExitCode). See $logPath" }
Write-Host "Test results: $xmlPath"
