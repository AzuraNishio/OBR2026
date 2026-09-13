import ast
import asyncio
import types


DEFAULT_ALLOWED_IMPORTS = {"umath", "math"}


class _ImportStripper(ast.NodeTransformer):
    """Drops any import not in `allowed_imports`, and strips type
    annotations (so e.g. MicroPython-style hints don't break exec)."""

    def __init__(self, allowed_imports):
        self.allowed_imports = allowed_imports

    def visit_Import(self, node):
        new_names = [n for n in node.names if n.name in self.allowed_imports]
        if not new_names:
            return None
        node.names = new_names
        return node

    def visit_ImportFrom(self, node):
        if not node.module or node.module.split(".")[0] not in self.allowed_imports:
            return None
        return node

    def visit_FunctionDef(self, node):
        for arg in (node.args.args + node.args.kwonlyargs):
            arg.annotation = None
        if node.args.vararg:
            node.args.vararg.annotation = None
        if node.args.kwarg:
            node.args.kwarg.annotation = None
        node.returns = None

        self.generic_visit(node)
        return node

    def visit_AsyncFunctionDef(self, node):
        return self.visit_FunctionDef(node)

    def visit_AnnAssign(self, node):
        if node.value is not None:
            new = ast.Assign(targets=[node.target], value=node.value)
            return ast.copy_location(new, node)
        return None


def _load_module(path, allowed_imports, module_name="room_module"):
    with open(path, "r", encoding="utf-8") as f:
        src = f.read()

    tree = ast.parse(src)
    tree = _ImportStripper(allowed_imports).visit(tree)
    ast.fix_missing_locations(tree)

    code_obj = compile(tree, path, "exec")

    module = types.ModuleType(module_name)
    exec(code_obj, module.__dict__)
    return module


async def runSimulation(path, entry_func="room3", allowed_imports=None, **kwargs):
    """
    Load the module at `path` with only `allowed_imports` surviving
    the AST pass, then call its `entry_func` with **kwargs.

    Whatever the entry function needs (a robot, a ctx, sensors...)
    gets passed in via kwargs by the caller - this file doesn't
    know or care what those are.
    """
    module = _load_module(path, allowed_imports or DEFAULT_ALLOWED_IMPORTS)

    func = getattr(module, entry_func, None)
    if func is None:
        raise AttributeError(f"No '{entry_func}' found in {path}")

    result = func(**kwargs)

    if hasattr(result, "__await__"):
        result = await result

    return result


def runSimulationSync(*args, **kwargs):
    """Convenience wrapper for callers that aren't already async."""
    return asyncio.run(runSimulation(*args, **kwargs))