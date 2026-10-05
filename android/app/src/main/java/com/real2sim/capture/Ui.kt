package com.real2sim.capture

import android.animation.ValueAnimator
import android.annotation.SuppressLint
import android.app.Activity
import android.app.Dialog
import android.content.Context
import android.content.res.ColorStateList
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.graphics.Typeface
import android.graphics.drawable.ColorDrawable
import android.graphics.drawable.Drawable
import android.graphics.drawable.GradientDrawable
import android.graphics.drawable.RippleDrawable
import android.text.InputType
import android.util.AttributeSet
import android.view.Gravity
import android.view.MotionEvent
import android.view.View
import android.view.ViewConfiguration
import android.view.ViewGroup
import android.view.WindowManager
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.widget.SwitchCompat
import androidx.core.content.ContextCompat
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import kotlin.math.abs
import kotlin.math.min

/**
 * Shared look of the app, modelled on the iPhone app (SwiftUI, dark): system colours, translucent
 * "material" cards, segmented controls, round icon buttons, the record button, inset-grouped forms in
 * bottom sheets, swipeable list rows. Everything is programmatic, no extra dependencies.
 */
object Ui {
    val BLUE = Color.rgb(10, 132, 255)
    val RED = Color.rgb(255, 59, 48)
    val GREEN = Color.rgb(48, 209, 88)
    val ORANGE = Color.rgb(255, 159, 10)
    val YELLOW = Color.rgb(255, 214, 10)
    val CYAN = Color.rgb(100, 210, 255)
    val PURPLE = Color.rgb(191, 90, 242)
    val GRAY = Color.rgb(142, 142, 147)
    val LABEL = Color.WHITE
    val SECONDARY = Color.argb(153, 235, 235, 245)
    val TERTIARY = Color.argb(77, 235, 235, 245)
    val SEPARATOR = Color.argb(140, 84, 84, 88)
    /** Grouped backgrounds as in iOS dark mode (sheet / cell / pressed). */
    val BG_SHEET = Color.rgb(28, 28, 30)
    val BG_CELL = Color.rgb(44, 44, 46)
    val BG_FILL = Color.argb(61, 118, 118, 128)
    /** ultraThinMaterial over a camera image, approximated (no live blur before API 31 on a View). */
    val MATERIAL = Color.argb(150, 28, 28, 30)

    val SEMIBOLD: Typeface = Typeface.create("sans-serif-medium", Typeface.NORMAL)
    val BOLD: Typeface = Typeface.create("sans-serif", Typeface.BOLD)
    val REGULAR: Typeface = Typeface.create("sans-serif", Typeface.NORMAL)
    /** SF ".monospacedDigit()": Roboto's tabular figures. */
    const val TNUM = "tnum"

    fun dp(ctx: Context, v: Float) = v * ctx.resources.displayMetrics.density
    fun dpi(ctx: Context, v: Int) = (v * ctx.resources.displayMetrics.density).toInt()

    fun rounded(color: Int, radiusPx: Float, strokeColor: Int = 0, strokePx: Int = 0) = GradientDrawable().apply {
        setColor(color); cornerRadius = radiusPx
        if (strokePx > 0) setStroke(strokePx, strokeColor)
    }

    fun oval(color: Int, strokeColor: Int = 0, strokePx: Int = 0) = GradientDrawable().apply {
        shape = GradientDrawable.OVAL; setColor(color)
        if (strokePx > 0) setStroke(strokePx, strokeColor)
    }

    /** Ripple on top of [content], masked to it. */
    fun pressable(content: Drawable, mask: Drawable? = content) =
        RippleDrawable(ColorStateList.valueOf(Color.argb(60, 255, 255, 255)), content, mask)

    /** Translucent rounded card (SwiftUI `.background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius:))`). */
    fun material(ctx: Context, radiusDp: Float = 16f) =
        rounded(MATERIAL, dp(ctx, radiusDp), Color.argb(30, 255, 255, 255), dpi(ctx, 1).coerceAtLeast(1))

    fun label(
        ctx: Context, text: CharSequence = "", sizeSp: Float = 15f, color: Int = LABEL,
        face: Typeface = REGULAR, mono: Boolean = false, digits: Boolean = false,
    ) = TextView(ctx).apply {
        this.text = text; textSize = sizeSp; setTextColor(color)
        typeface = if (mono) Typeface.MONOSPACE else face
        if (digits) fontFeatureSettings = TNUM
        includeFontPadding = false
    }

    fun icon(ctx: Context, res: Int, tint: Int = LABEL, sizeDp: Int = 24) = ImageView(ctx).apply {
        setImageDrawable(ContextCompat.getDrawable(ctx, res)?.mutate())
        imageTintList = ColorStateList.valueOf(tint)
        layoutParams = LinearLayout.LayoutParams(dpi(ctx, sizeDp), dpi(ctx, sizeDp))
    }

    /** Round translucent icon button (folder, gear, ... next to the record button). */
    fun iconButton(ctx: Context, res: Int, desc: String, sizeDp: Int = 48, tint: Int = LABEL, bg: Int = MATERIAL) = FrameLayout(ctx).apply {
        background = pressable(oval(bg, Color.argb(30, 255, 255, 255), 1), oval(Color.WHITE))
        contentDescription = desc
        isClickable = true; isFocusable = true
        addView(ImageView(ctx).apply {
            setImageDrawable(ContextCompat.getDrawable(ctx, res)?.mutate())
            imageTintList = ColorStateList.valueOf(tint)
        }, FrameLayout.LayoutParams(dpi(ctx, sizeDp * 11 / 24), dpi(ctx, sizeDp * 11 / 24), Gravity.CENTER))
        layoutParams = LinearLayout.LayoutParams(dpi(ctx, sizeDp), dpi(ctx, sizeDp))
    }

    /** Filled rounded button (GO, STOP, prominent actions). */
    fun filledButton(ctx: Context, text: String, color: Int, onClick: () -> Unit) = label(ctx, text, 17f, Color.WHITE, BOLD).apply {
        gravity = Gravity.CENTER
        minHeight = dpi(ctx, 44)
        setPadding(dpi(ctx, 14), 0, dpi(ctx, 14), 0)
        background = pressable(rounded(color, dp(ctx, 12f)))
        setOnClickListener { onClick() }
    }

    /** SwiftUI `.buttonStyle(.bordered)`: tinted text on a translucent grey capsule-ish rect. */
    fun borderedButton(ctx: Context, text: String, tint: Int = BLUE, onClick: () -> Unit) = label(ctx, text, 15f, tint, SEMIBOLD).apply {
        gravity = Gravity.CENTER
        minHeight = dpi(ctx, 36)
        setPadding(dpi(ctx, 12), 0, dpi(ctx, 12), 0)
        background = pressable(rounded(BG_FILL, dp(ctx, 8f)))
        setOnClickListener { onClick() }
    }

    /** iOS toggle look on an AppCompat switch. */
    fun toggle(ctx: Context, on: Boolean, onChange: (Boolean) -> Unit) = SwitchCompat(ctx).apply {
        trackDrawable = ContextCompat.getDrawable(ctx, R.drawable.track_ios)
        thumbDrawable = ContextCompat.getDrawable(ctx, R.drawable.thumb_ios)
        trackTintList = null; thumbTintList = null
        showText = false
        isChecked = on
        setOnCheckedChangeListener { _, c -> onChange(c) }
    }

    /** Pads [v] by the system bars (edge-to-edge window): top and/or bottom, keeping its own padding. */
    fun applyInsets(v: View, top: Boolean, bottom: Boolean, sides: Boolean = true) {
        val l = v.paddingLeft; val t = v.paddingTop; val r = v.paddingRight; val b = v.paddingBottom
        ViewCompat.setOnApplyWindowInsetsListener(v) { view, ins ->
            val i = ins.getInsets(WindowInsetsCompat.Type.systemBars() or WindowInsetsCompat.Type.displayCutout())
            val ime = ins.getInsets(WindowInsetsCompat.Type.ime())
            view.setPadding(l + if (sides) i.left else 0, t + if (top) i.top else 0,
                r + if (sides) i.right else 0, b + if (bottom) maxOf(i.bottom, ime.bottom) else 0)
            ins
        }
        ViewCompat.requestApplyInsets(v)
    }

    /** Black, edge-to-edge window with light system-bar icons (all screens are dark like the iOS app). */
    fun edgeToEdge(a: androidx.activity.ComponentActivity) {
        a.enableEdgeToEdge(
            androidx.activity.SystemBarStyle.dark(Color.TRANSPARENT),
            androidx.activity.SystemBarStyle.dark(Color.TRANSPARENT),
        )
    }

    fun fmtClock(ms: Long): String {
        val s = (ms / 1000).coerceAtLeast(0)
        return if (s >= 3600) "%d:%02d:%02d".format(s / 3600, s / 60 % 60, s % 60) else "%02d:%02d".format(s / 60, s % 60)
    }

    /** Navigation-bar header of a pushed / sheet screen: [leading] · centred title · [trailing]. */
    fun navBar(ctx: Context, title: String, leading: View?, trailing: View?): FrameLayout = FrameLayout(ctx).apply {
        minimumHeight = dpi(ctx, 44)
        setPadding(dpi(ctx, 8), 0, dpi(ctx, 8), 0)
        addView(label(ctx, title, 17f, LABEL, SEMIBOLD).apply {
            gravity = Gravity.CENTER; maxLines = 1; ellipsize = android.text.TextUtils.TruncateAt.MIDDLE
            setPadding(dpi(ctx, 112), 0, dpi(ctx, 112), 0)
        }, FrameLayout.LayoutParams(-1, dpi(ctx, 44), Gravity.CENTER))
        leading?.let { addView(it, FrameLayout.LayoutParams(-2, dpi(ctx, 44), Gravity.START or Gravity.CENTER_VERTICAL)) }
        trailing?.let { addView(it, FrameLayout.LayoutParams(-2, dpi(ctx, 44), Gravity.END or Gravity.CENTER_VERTICAL)) }
    }

    /** Plain blue text button for nav bars ("Done", "‹ Sessions"). */
    fun barButton(ctx: Context, text: String, bold: Boolean = false, chevron: Boolean = false, onClick: () -> Unit): View {
        val row = LinearLayout(ctx).apply {
            orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
            setPadding(dpi(ctx, 8), 0, dpi(ctx, 8), 0)
            isClickable = true; isFocusable = true
            background = pressable(ColorDrawable(Color.TRANSPARENT), rounded(Color.WHITE, dp(ctx, 8f)))
            setOnClickListener { onClick() }
            contentDescription = text
        }
        if (chevron) row.addView(icon(ctx, R.drawable.ic_chevron_left, BLUE, 20))
        row.addView(label(ctx, text, 17f, BLUE, if (bold) SEMIBOLD else REGULAR))
        return row
    }

    fun alert(ctx: Context) = AlertDialog.Builder(ctx)

    /** Long text in a dialog (camera inventory, errors) with selectable monospaced text. */
    fun showText(ctx: Context, title: String, text: String) {
        val tv = label(ctx, text, 11f, LABEL, mono = true).apply {
            setTextIsSelectable(true); setPadding(dpi(ctx, 20), dpi(ctx, 8), dpi(ctx, 20), dpi(ctx, 8))
        }
        alert(ctx).setTitle(title).setView(ScrollView(ctx).apply { addView(tv) }).setPositiveButton("OK", null).show()
    }
}

// ---------------------------------------------------------------------------------------------- views

/** UISegmentedControl look-alike. */
class SegmentedControl @JvmOverloads constructor(ctx: Context, attrs: AttributeSet? = null) : LinearLayout(ctx, attrs) {
    var onSelect: ((Int) -> Unit)? = null
    private val segs = ArrayList<TextView>()
    var selected = 0
        private set

    init {
        orientation = HORIZONTAL
        background = Ui.rounded(Ui.BG_FILL, Ui.dp(ctx, 9f))
        val p = Ui.dpi(ctx, 2)
        setPadding(p, p, p, p)
        minimumHeight = Ui.dpi(ctx, 34)
    }

    fun setItems(items: List<String>, sel: Int = 0) {
        removeAllViews(); segs.clear()
        items.forEachIndexed { i, s ->
            val tv = Ui.label(context, s, 13f, Ui.LABEL, Ui.REGULAR).apply {
                gravity = Gravity.CENTER; maxLines = 1; ellipsize = android.text.TextUtils.TruncateAt.END
                setPadding(Ui.dpi(context, 6), 0, Ui.dpi(context, 6), 0)
                isClickable = true; isFocusable = true
                contentDescription = s
                setOnClickListener { if (isEnabled) { select(i); onSelect?.invoke(i) } }
            }
            segs.add(tv)
            addView(tv, LayoutParams(0, Ui.dpi(context, 30), 1f))
        }
        select(sel)
    }

    fun select(i: Int) {
        selected = i.coerceIn(0, maxOf(0, segs.size - 1))
        segs.forEachIndexed { k, tv ->
            val on = k == selected
            tv.background = if (on) Ui.rounded(Color.rgb(99, 99, 102), Ui.dp(context, 7f)) else null
            tv.typeface = if (on) Ui.SEMIBOLD else Ui.REGULAR
            tv.isSelected = on
        }
    }

    override fun setEnabled(enabled: Boolean) {
        super.setEnabled(enabled)
        segs.forEach { it.isEnabled = enabled }
        alpha = if (enabled) 1f else 0.45f
    }
}

/**
 * The iOS camera shutter: white ring, red disc that morphs into a rounded square while recording.
 */
class RecordButton @JvmOverloads constructor(ctx: Context, attrs: AttributeSet? = null) : View(ctx, attrs) {
    private val ring = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; color = Color.WHITE; strokeWidth = Ui.dp(ctx, 4f) }
    private val fill = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Ui.RED }
    private var f = 0f                  // 0 = idle disc, 1 = recording square
    private var anim: ValueAnimator? = null
    private val r = RectF()

    var recording = false
        set(v) {
            if (field == v) return
            field = v
            anim?.cancel()
            anim = ValueAnimator.ofFloat(f, if (v) 1f else 0f).apply {
                duration = 220
                addUpdateListener { f = it.animatedValue as Float; invalidate() }
                start()
            }
            contentDescription = if (v) "Stop recording" else "Record"
        }

    init {
        isClickable = true; isFocusable = true
        contentDescription = "Record"
    }

    override fun onMeasure(w: Int, h: Int) {
        val s = Ui.dpi(context, 76)
        setMeasuredDimension(resolveSize(s, w), resolveSize(s, h))
    }

    override fun onDraw(c: Canvas) {
        val cx = width / 2f; val cy = height / 2f
        val d = Ui.dp(context, 1f)
        val outer = min(width, height) / 2f
        c.drawCircle(cx, cy, outer - 2 * d, ring)
        val side = (62 + (34 - 62) * f) * d * (outer / (38 * d))
        val corner = (31 + (8 - 31) * f) * d * (outer / (38 * d))
        r.set(cx - side / 2, cy - side / 2, cx + side / 2, cy + side / 2)
        c.drawRoundRect(r, corner, corner, fill)
    }

    override fun setEnabled(enabled: Boolean) {
        super.setEnabled(enabled)
        alpha = if (enabled) 1f else 0.4f
    }

    override fun dispatchSetPressed(pressed: Boolean) { invalidate() }

    override fun onDrawForeground(canvas: Canvas) {
        super.onDrawForeground(canvas)
        if (isPressed) canvas.drawCircle(width / 2f, height / 2f, min(width, height) / 2f, Paint().apply { color = Color.argb(40, 255, 255, 255) })
    }
}

/**
 * List row with iOS swipe actions: drag left to reveal the action buttons; tap a row to open it.
 */
@SuppressLint("ViewConstructor")
class SwipeRow(ctx: Context, val content: View, actions: List<Triple<String, Int, () -> Unit>>) : FrameLayout(ctx) {
    private val actionsView = LinearLayout(ctx).apply { orientation = LinearLayout.HORIZONTAL }
    private val actionW = Ui.dpi(ctx, 76)
    private val maxOpen get() = actionW * actionsView.childCount
    private val slop = ViewConfiguration.get(ctx).scaledTouchSlop
    private var downX = 0f; private var downY = 0f; private var startTx = 0f
    private var dragging = false

    init {
        for ((label, color, action) in actions) {
            actionsView.addView(Ui.label(ctx, label, 15f, Color.WHITE, Ui.SEMIBOLD).apply {
                gravity = Gravity.CENTER
                setBackgroundColor(color)
                isClickable = true
                setOnClickListener { close(); action() }
            }, LinearLayout.LayoutParams(actionW, -1))
        }
        addView(actionsView, LayoutParams(-2, -1, Gravity.END))
        addView(content, LayoutParams(-1, -2))
        actionsView.visibility = INVISIBLE
    }

    fun close() { content.animate().translationX(0f).setDuration(180).withEndAction { actionsView.visibility = INVISIBLE }.start() }
    private fun open() { actionsView.visibility = VISIBLE; content.animate().translationX(-maxOpen.toFloat()).setDuration(180).start() }

    override fun onInterceptTouchEvent(e: MotionEvent): Boolean {
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> { downX = e.x; downY = e.y; startTx = content.translationX; dragging = false }
            MotionEvent.ACTION_MOVE -> {
                val dx = e.x - downX; val dy = e.y - downY
                if (!dragging && abs(dx) > slop && abs(dx) > abs(dy) * 1.5f) {
                    dragging = true; parent?.requestDisallowInterceptTouchEvent(true)
                    actionsView.visibility = VISIBLE
                }
            }
        }
        return dragging
    }

    @SuppressLint("ClickableViewAccessibility")
    override fun onTouchEvent(e: MotionEvent): Boolean {
        when (e.actionMasked) {
            MotionEvent.ACTION_MOVE -> if (dragging) content.translationX = (startTx + e.x - downX).coerceIn(-maxOpen.toFloat() * 1.15f, 0f)
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                if (dragging) { if (content.translationX < -maxOpen / 2f) open() else close() }
                dragging = false
            }
        }
        return true
    }

    override fun dispatchTouchEvent(e: MotionEvent): Boolean {
        // a tap on an open row closes it instead of opening the session
        if (e.actionMasked == MotionEvent.ACTION_DOWN && content.translationX < -1f && e.x < width - maxOpen) { close(); return false }
        return super.dispatchTouchEvent(e)
    }
}

// ---------------------------------------------------------------------------------------------- forms

/**
 * Inset-grouped form (SwiftUI `Form` / `Section`) used inside [Sheet]s: section headers in small grey
 * caps, rounded cells with hairline separators, grey footers.
 */
class Form(private val ctx: Context) {
    val root = LinearLayout(ctx).apply {
        orientation = LinearLayout.VERTICAL
        setPadding(Ui.dpi(ctx, 16), 0, Ui.dpi(ctx, 16), Ui.dpi(ctx, 24))
    }

    fun section(header: String?, footer: String? = null, body: Section.() -> Unit): Section {
        header?.let {
            root.addView(Ui.label(ctx, it.uppercase(), 12f, Ui.SECONDARY).apply {
                setPadding(Ui.dpi(ctx, 16), Ui.dpi(ctx, 22), Ui.dpi(ctx, 16), Ui.dpi(ctx, 7))
            })
        } ?: root.addView(View(ctx), LinearLayout.LayoutParams(1, Ui.dpi(ctx, 20)))
        val s = Section(ctx)
        s.body()
        root.addView(s.card, LinearLayout.LayoutParams(-1, -2))
        footer?.let {
            root.addView(Ui.label(ctx, it, 12f, Ui.SECONDARY).apply {
                setPadding(Ui.dpi(ctx, 16), Ui.dpi(ctx, 7), Ui.dpi(ctx, 16), 0)
                setLineSpacing(0f, 1.15f)
            })
        }
        return s
    }

    class Section(private val ctx: Context) {
        val card = LinearLayout(ctx).apply {
            orientation = LinearLayout.VERTICAL
            background = Ui.rounded(Ui.BG_CELL, Ui.dp(ctx, 10f))
            clipToOutline = true
        }

        private fun row(): LinearLayout {
            if (card.childCount > 0) card.addView(View(ctx).apply { setBackgroundColor(Ui.SEPARATOR) },
                LinearLayout.LayoutParams(-1, maxOf(1, Ui.dpi(ctx, 1) / 2)).apply { marginStart = Ui.dpi(ctx, 16) })
            val r = LinearLayout(ctx).apply {
                orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
                minimumHeight = Ui.dpi(ctx, 44)
                setPadding(Ui.dpi(ctx, 16), Ui.dpi(ctx, 6), Ui.dpi(ctx, 12), Ui.dpi(ctx, 6))
            }
            card.addView(r, LinearLayout.LayoutParams(-1, -2))
            return r
        }

        private fun title(r: LinearLayout, text: String, color: Int = Ui.LABEL, sub: String? = null) {
            val col = LinearLayout(ctx).apply { orientation = LinearLayout.VERTICAL }
            col.addView(Ui.label(ctx, text, 16f, color))
            sub?.let { col.addView(Ui.label(ctx, it, 12f, Ui.SECONDARY).apply { setPadding(0, Ui.dpi(ctx, 2), 0, 0) }) }
            r.addView(col, LinearLayout.LayoutParams(0, -2, 1f))
        }

        fun toggle(text: String, on: Boolean, sub: String? = null, onChange: (Boolean) -> Unit): SwitchCompat {
            val r = row(); title(r, text, sub = sub)
            val sw = Ui.toggle(ctx, on, onChange)
            sw.contentDescription = text
            r.addView(sw)
            r.setOnClickListener { sw.toggle() }
            return sw
        }

        fun field(hint: String, value: String, input: Int = InputType.TYPE_CLASS_TEXT, onChange: (String) -> Unit): EditText {
            val r = row()
            val e = EditText(ctx).apply {
                this.hint = hint; setText(value); inputType = input; setSingleLine()
                textSize = 16f; setTextColor(Ui.LABEL); setHintTextColor(Ui.TERTIARY)
                background = null; setPadding(0, 0, 0, 0)
                addTextChangedListener(object : android.text.TextWatcher {
                    override fun afterTextChanged(s: android.text.Editable?) { onChange(s.toString()) }
                    override fun beforeTextChanged(a: CharSequence?, b: Int, c: Int, d: Int) {}
                    override fun onTextChanged(a: CharSequence?, b: Int, c: Int, d: Int) {}
                })
            }
            r.addView(e, LinearLayout.LayoutParams(0, -2, 1f))
            return e
        }

        /** Labelled field ("Robot radius  [ 20 ] cm"), numbers right-aligned like iOS value cells. */
        fun number(text: String, value: String, unit: String, onChange: (String) -> Unit): EditText {
            val r = row(); title(r, text)
            val e = EditText(ctx).apply {
                setText(value); setSingleLine(); gravity = Gravity.END
                inputType = InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL or InputType.TYPE_NUMBER_FLAG_SIGNED
                textSize = 16f; setTextColor(Ui.BLUE); fontFeatureSettings = Ui.TNUM
                background = null; setPadding(Ui.dpi(ctx, 8), 0, Ui.dpi(ctx, 4), 0); minWidth = Ui.dpi(ctx, 56)
                contentDescription = text
                addTextChangedListener(object : android.text.TextWatcher {
                    override fun afterTextChanged(s: android.text.Editable?) { onChange(s.toString()) }
                    override fun beforeTextChanged(a: CharSequence?, b: Int, c: Int, d: Int) {}
                    override fun onTextChanged(a: CharSequence?, b: Int, c: Int, d: Int) {}
                })
            }
            r.addView(e)
            r.addView(Ui.label(ctx, unit, 16f, Ui.SECONDARY))
            return e
        }

        fun button(text: String, color: Int = Ui.BLUE, onClick: () -> Unit): TextView {
            val r = row()
            val t = Ui.label(ctx, text, 16f, color)
            r.addView(t, LinearLayout.LayoutParams(0, -2, 1f))
            r.isClickable = true
            r.background = Ui.pressable(ColorDrawable(Color.TRANSPARENT), ColorDrawable(Color.WHITE))
            r.setOnClickListener { onClick() }
            return t
        }

        /** Read-only "label ... value" cell. */
        fun value(text: String, value: String): TextView {
            val r = row(); title(r, text)
            val v = Ui.label(ctx, value, 16f, Ui.SECONDARY, digits = true).apply { gravity = Gravity.END }
            r.addView(v)
            return v
        }

        /** Menu picker cell: current choice in grey with a chevron; opens a single-choice list. */
        fun choice(text: String, options: List<String>, sel: Int, onChange: (Int) -> Unit): TextView {
            val r = row(); title(r, text)
            var cur = sel
            val v = Ui.label(ctx, options.getOrElse(sel) { "" }, 16f, Ui.SECONDARY).apply { gravity = Gravity.END; maxLines = 1 }
            r.addView(v, LinearLayout.LayoutParams(-2, -2).apply { marginStart = Ui.dpi(ctx, 8) })
            r.addView(Ui.icon(ctx, R.drawable.ic_chevron_down, Ui.SECONDARY, 16).apply {
                (layoutParams as LinearLayout.LayoutParams).marginStart = Ui.dpi(ctx, 4)
            })
            r.isClickable = true
            r.background = Ui.pressable(ColorDrawable(Color.TRANSPARENT), ColorDrawable(Color.WHITE))
            r.setOnClickListener {
                Ui.alert(ctx).setTitle(text).setSingleChoiceItems(options.toTypedArray(), cur) { d, i ->
                    cur = i; v.text = options[i]; onChange(i); d.dismiss()
                }.show()
            }
            return v
        }

        /** SwiftUI inline picker: one row per option, a blue check on the selected one. */
        fun inlinePicker(options: List<String>, sel: Int, onChange: (Int) -> Unit) {
            val checks = ArrayList<ImageView>()
            options.forEachIndexed { i, o ->
                val r = row()
                r.addView(Ui.label(ctx, o, 16f), LinearLayout.LayoutParams(0, -2, 1f))
                val chk = ImageView(ctx).apply {
                    setImageDrawable(ContextCompat.getDrawable(ctx, R.drawable.ic_check)?.mutate())
                    imageTintList = ColorStateList.valueOf(Ui.BLUE)
                    visibility = if (i == sel) View.VISIBLE else View.INVISIBLE
                }
                checks.add(chk)
                r.addView(chk, LinearLayout.LayoutParams(Ui.dpi(ctx, 20), Ui.dpi(ctx, 20)))
                r.isClickable = true
                r.background = Ui.pressable(ColorDrawable(Color.TRANSPARENT), ColorDrawable(Color.WHITE))
                r.setOnClickListener {
                    checks.forEachIndexed { k, c -> c.visibility = if (k == i) View.VISIBLE else View.INVISIBLE }
                    onChange(i)
                }
            }
        }

        /** Stepper cell ("Bitrate 50 Mbps   [ − | + ]"). */
        fun stepper(text: (Float) -> String, value: Float, minV: Float, maxV: Float, step: Float, onChange: (Float) -> Unit) {
            val r = row()
            var v = value
            val t = Ui.label(ctx, text(v), 16f, digits = true)
            r.addView(t, LinearLayout.LayoutParams(0, -2, 1f))
            val box = LinearLayout(ctx).apply {
                orientation = LinearLayout.HORIZONTAL
                background = Ui.rounded(Ui.BG_FILL, Ui.dp(ctx, 8f))
            }
            fun b(s: String, d: Float) = Ui.label(ctx, s, 20f, Ui.LABEL).apply {
                gravity = Gravity.CENTER
                isClickable = true
                background = Ui.pressable(ColorDrawable(Color.TRANSPARENT), Ui.rounded(Color.WHITE, Ui.dp(ctx, 8f)))
                contentDescription = if (d < 0) "decrease" else "increase"
                setOnClickListener { v = (v + d).coerceIn(minV, maxV); t.text = text(v); onChange(v) }
            }
            box.addView(b("−", -step), LinearLayout.LayoutParams(Ui.dpi(ctx, 46), Ui.dpi(ctx, 32)))
            box.addView(View(ctx).apply { setBackgroundColor(Ui.SEPARATOR) }, LinearLayout.LayoutParams(Ui.dpi(ctx, 1), Ui.dpi(ctx, 18)).apply { gravity = Gravity.CENTER_VERTICAL })
            box.addView(b("+", step), LinearLayout.LayoutParams(Ui.dpi(ctx, 46), Ui.dpi(ctx, 32)))
            r.addView(box)
        }

        /** Any custom content in its own cell. */
        fun custom(v: View) { row().addView(v, LinearLayout.LayoutParams(0, -2, 1f)) }

        /** Grey caption inside the card. */
        fun note(text: String): TextView {
            val r = row()
            val t = Ui.label(ctx, text, 12f, Ui.SECONDARY).apply { setLineSpacing(0f, 1.15f) }
            r.addView(t, LinearLayout.LayoutParams(0, -2, 1f))
            return t
        }
    }
}

/**
 * iOS-style sheet: slides up over the screen, grabber, centred title and a "Done" button, scrolling
 * content. Back / tap on the dimmed area above closes it.
 */
class Sheet(private val act: Activity, title: String, heightFraction: Float = 0.92f) {
    val dialog = Dialog(act, R.style.Theme_Capture_Sheet)
    val body = LinearLayout(act).apply { orientation = LinearLayout.VERTICAL }
    var onDismiss: (() -> Unit)? = null

    init {
        val screenH = act.resources.displayMetrics.heightPixels
        val outer = FrameLayout(act)
        outer.setOnClickListener { dialog.dismiss() }
        val panel = LinearLayout(act).apply {
            orientation = LinearLayout.VERTICAL
            val r = Ui.dp(act, 12f)
            background = GradientDrawable().apply { setColor(Ui.BG_SHEET); cornerRadii = floatArrayOf(r, r, r, r, 0f, 0f, 0f, 0f) }
            isClickable = true      // swallow taps so they don't reach `outer`
        }
        panel.addView(View(act).apply { background = Ui.rounded(Color.argb(110, 235, 235, 245), Ui.dp(act, 3f)) },
            LinearLayout.LayoutParams(Ui.dpi(act, 36), Ui.dpi(act, 5)).apply { gravity = Gravity.CENTER_HORIZONTAL; topMargin = Ui.dpi(act, 6) })
        panel.addView(Ui.navBar(act, title, null, Ui.barButton(act, "Done", bold = true) { dialog.dismiss() }))
        panel.addView(ScrollView(act).apply { addView(body); isFillViewport = true }, LinearLayout.LayoutParams(-1, 0, 1f))
        outer.addView(panel, FrameLayout.LayoutParams(-1, (screenH * heightFraction).toInt(), Gravity.BOTTOM))
        Ui.applyInsets(panel, top = false, bottom = true)
        dialog.setContentView(outer)
        dialog.window?.apply {
            setLayout(WindowManager.LayoutParams.MATCH_PARENT, WindowManager.LayoutParams.MATCH_PARENT)
            androidx.core.view.WindowCompat.setDecorFitsSystemWindows(this, false)
        }
        dialog.setOnDismissListener { onDismiss?.invoke() }
    }

    fun form(build: Form.() -> Unit): Sheet {
        val f = Form(act); f.build(); body.addView(f.root, LinearLayout.LayoutParams(-1, -2)); return this
    }

    fun show(): Sheet { dialog.show(); return this }
    fun dismiss() = dialog.dismiss()
}

/** Group of child views laid out with a fixed gap (SwiftUI HStack(spacing:)). */
fun hstack(ctx: Context, gapDp: Int, vararg views: View, gravity: Int = Gravity.CENTER_VERTICAL) = LinearLayout(ctx).apply {
    orientation = LinearLayout.HORIZONTAL; this.gravity = gravity
    views.forEachIndexed { i, v ->
        val lp = (v.layoutParams as? LinearLayout.LayoutParams) ?: LinearLayout.LayoutParams(-2, -2)
        if (i > 0) lp.marginStart = Ui.dpi(ctx, gapDp)
        addView(v, lp)
    }
}

internal fun ViewGroup.addAll(vararg v: View) { v.forEach { addView(it) } }
