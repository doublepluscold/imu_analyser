"""Friendly text for connection errors. The raw exception goes to "Деталі", not to the banner."""

import errno


def friendly(exc: BaseException, transport: str = "", target: str = "") -> tuple[str, str]:
    """(message for the user, raw details). transport: serial | udp | sim."""
    raw = f"{type(exc).__name__}: {exc}"
    text = str(exc).lower()
    eno = getattr(exc, "errno", None)
    where = f" {target}" if target else ""
    if transport == "serial":
        if eno in (errno.EACCES, errno.EPERM) or "permission" in text:
            msg = (f"Немає доступу до порту{where}. На Linux додай себе в групу dialout "
                   "і перелогінься.")  # fmt: skip
        elif "busy" in text or eno == errno.EBUSY or "could not exclusively lock" in text:
            msg = f"Порт{where} зайнятий іншою програмою. Закрий її і спробуй знову."
        elif (
            eno in (errno.ENOENT, errno.ENODEV)
            or "no such file" in text
            or "could not open" in text
        ):
            msg = f"Порт{where} не знайдено. Перевір кабель і натисни «оновити список»."
        else:
            msg = f"Не вдалося відкрити порт{where}."
    elif transport == "udp":
        if eno == errno.EADDRINUSE or "address already in use" in text:
            msg = f"UDP-порт{where} уже зайнятий іншою програмою."
        elif eno in (errno.EACCES, errno.EPERM):
            msg = f"Немає дозволу слухати UDP-порт{where}."
        elif isinstance(exc, ValueError):
            msg = "Порт має бути числом від 1 до 65535."
        else:
            msg = f"Не вдалося почати слухати UDP{where}."
    else:
        msg = "Не вдалося під'єднатися."
    return msg, raw
