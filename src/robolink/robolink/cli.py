import asyncio
import typer
from pathlib import Path

app = typer.Typer(
    help="robolink — async Python SDK for industrial robot arm control",
    add_completion=False,
)


@app.command()
def run(
    script: Path = typer.Argument(..., help="Path to .vas script file"),
    step_delay: float = typer.Option(
        0.6, "--delay", "-d", help="Default seconds between moves"
    ),
    speed: float = typer.Option(
        1.0, "--speed", "-s", help="Speed multiplier 0.0–1.0"
    ),
):
    """Execute a V-Alpha Script (.vas) file."""
    from robolink.client import ArmClient
    from robolink.script import ScriptRunner, ScriptError

    if not script.exists():
        typer.echo(f"Error: file not found: {script}", err=True)
        raise typer.Exit(1)

    async def _run():
        async with ArmClient(step_delay=step_delay) as arm:
            runner = ScriptRunner(arm, speed=speed)
            try:
                await runner.run_file(script)
            except ScriptError as e:
                typer.echo(f"Script error: {e}", err=True)
                raise typer.Exit(1)
            except NotImplementedError as e:
                typer.echo(f"Not implemented: {e}", err=True)
                raise typer.Exit(1)

    asyncio.run(_run())


@app.command()
def validate(
    script: Path = typer.Argument(..., help="Path to .vas script file"),
):
    """Validate a .vas script without running it."""
    from robolink.script import ScriptParser, ScriptError

    if not script.exists():
        typer.echo(f"Error: file not found: {script}", err=True)
        raise typer.Exit(1)

    source = script.read_text()
    try:
        commands = ScriptParser().parse(source)
        typer.echo(f"✓ Valid — {len(commands)} top-level commands")
    except ScriptError as e:
        typer.echo(f"✗ {e}", err=True)
        raise typer.Exit(1)


if __name__ == "__main__":
    app()